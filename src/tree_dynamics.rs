// tree_dynamics.rs — TreeDynamicsModel, rigid-body dynamics for the floating-base tree: mass matrix,
// inverse dynamics, gravity and bias over nv = 6 + n_q generalized velocities [omega, v, qd]. The
// recursion is the fixed-base RNEA lifted to a tree (gravity through a0 = +g), M = sum_links
// J_i' I_i J_i with the armature on the diagonal; ID identities live in tests/tree_dynamics.rs.

use crate::body_tree::BodyTree;
use crate::pga_dynamics::pga_screw_bracket_angular;
use crate::pga_layer::{pga_biv_to_axial, pga_vec_to_biv};
use control_math::mat::Mat;
use control_math::quat::Quat;
use control_math::vec3::Vec3;
use std::sync::Arc;

/// The traits are what a HOLDER needs; the clone REBUILDS rather than copying it (see `clone_shallow`).
#[derive(Debug)]
pub struct TreeDynamicsModel {
    pub nv: usize,
    /// the model's tree, behind an `Arc` so the holders of one machine SHARE it.
    pub tree: Arc<BodyTree>,
    fr_q: Vec<f64>,
    fr_base_p: Vec3,
    fr_base_q: Quat,
    fr_o: Vec<Vec3>,
    fr_r: Vec<Mat>,
    fr_iw: Vec<Mat>,
    fr_valid: bool,
    /// mass matrix scratch: the 6 x nv Jacobian and the walk path (see examples/alloc_count_probe.rs).
    sc_j: Mat,
    sc_path: Vec<usize>,
    /// inverse dynamics scratch: the pass vectors and the buffers the biases are built from.
    sc_om: Vec<Vec3>,
    sc_al: Vec<Vec3>,
    sc_ao: Vec<Vec3>,
    sc_f: Vec<Vec3>,
    sc_n: Vec<Vec3>,
    sc_zero: Vec<f64>,
    sc_id: Vec<f64>,
    /// the frame pass's two intermediate 3 x 3 matrices (parent rotation product, joint rotation).
    sc_t1: Mat,
    sc_t2: Mat,
}

impl TreeDynamicsModel {
    pub fn new(tree: impl Into<Arc<BodyTree>>) -> TreeDynamicsModel {
        let tree: Arc<BodyTree> = tree.into();
        let nv = tree.nv();
        TreeDynamicsModel {
            nv,
            tree,
            fr_q: Vec::new(),
            fr_base_p: Vec3::default(),
            fr_base_q: Quat::IDENTITY,
            fr_o: Vec::new(),
            fr_r: Vec::new(),
            fr_iw: Vec::new(),
            fr_valid: false,
            sc_j: Mat::zeros(0, 0),
            sc_path: Vec::new(),
            sc_om: Vec::new(),
            sc_al: Vec::new(),
            sc_ao: Vec::new(),
            sc_f: Vec::new(),
            sc_n: Vec::new(),
            sc_zero: Vec::new(),
            sc_id: Vec::new(),
            sc_t1: Mat::zeros(0, 0),
            sc_t2: Mat::zeros(0, 0),
        }
    }

    /// clone_shallow is a fresh model on the same tree with the frame cache dropped: the cache is a
    /// (base, q)-keyed memo, so a clone carrying it would be someone else's state.
    pub fn clone_shallow(&self) -> TreeDynamicsModel {
        TreeDynamicsModel::new(Arc::clone(&self.tree))
    }

    fn frames(&mut self, base_p: Vec3, base_q: Quat, q: &[f64]) {
        if self.fr_valid
            && self.fr_q.len() == q.len()
            && self.fr_base_p == base_p
            && self.fr_base_q == base_q
        {
            let mut same = true;
            for i in 0..q.len() {
                if self.fr_q[i] != q[i] {
                    same = false;
                    break;
                }
            }
            if same {
                return;
            }
        }
        self.fr_q = q.to_vec();
        self.fr_base_p = base_p;
        self.fr_base_q = base_q;
        // the FK pass writes into this model's own frame buffers and scratch.
        self.tree.fk_into(
            base_p,
            base_q,
            q,
            &mut self.fr_o,
            &mut self.fr_r,
            (&mut self.sc_t1, &mut self.sc_t2),
        );
        self.fr_iw
            .resize_with(self.tree.nodes.len(), || Mat::zeros(3, 3));
        for i in 0..self.tree.nodes.len() {
            // I_w = R I R'
            self.fr_r[i].mul_into(&self.tree.nodes[i].ic, &mut self.sc_t1);
            self.fr_r[i].transposed_into(&mut self.sc_t2);
            self.sc_t1.mul_into(&self.sc_t2, &mut self.fr_iw[i]);
        }
        self.fr_valid = true;
    }

    /// refresh_frames is the cache's own refresh, for a caller that needs the frames about to be used.
    pub fn refresh_frames(&mut self, base_p: Vec3, base_q: Quat, q: &[f64]) {
        self.frames(base_p, base_q, q);
    }

    /// frames_now lends out the cache's frames: the state `refresh_frames` was last given.
    pub fn frames_now(&self) -> (&[Vec3], &[Mat]) {
        (&self.fr_o, &self.fr_r)
    }

    /// mass_matrix: M = sum_links J_lin' m J_lin + J_ang' I_w J_ang over nv, plus the armature.
    pub fn mass_matrix(&mut self, base_p: Vec3, base_q: Quat, q: &[f64]) -> Mat {
        let mut m = Mat::zeros(self.nv, self.nv);
        self.mass_matrix_into(base_p, base_q, q, &mut m);
        m
    }

    pub fn mass_matrix_into(&mut self, base_p: Vec3, base_q: Quat, q: &[f64], m: &mut Mat) {
        self.frames(base_p, base_q, q);
        let nv = self.nv;
        if m.rows != nv || m.cols != nv {
            *m = Mat::zeros(nv, nv);
        } else {
            for v in m.data.iter_mut() {
                *v = 0.0;
            }
        }
        for i in 0..self.tree.nodes.len() {
            if self.tree.nodes[i].mass == 0.0 {
                continue;
            }
            self.tree.link_spatial_com_jacobian_into(
                &self.fr_o,
                &self.fr_r,
                i,
                &mut self.sc_j,
                &mut self.sc_path,
            );
            let j = &self.sc_j;
            let node_mass = self.tree.nodes[i].mass;
            // angular block: J_ang' I_w J_ang; linear block: m J_lin' J_lin
            for a in 0..nv {
                let mut ia = [0.0f64; 3];
                for k in 0..3 {
                    ia[k] = self.fr_iw[i].at(k, 0) * j.at(0, a)
                        + self.fr_iw[i].at(k, 1) * j.at(1, a)
                        + self.fr_iw[i].at(k, 2) * j.at(2, a);
                }
                for b in a..nv {
                    let mut s = 0.0;
                    for k in 0..3 {
                        s += j.at(k, b) * ia[k] + node_mass * j.at(3 + k, b) * j.at(3 + k, a);
                    }
                    m.set(a, b, m.at(a, b) + s);
                    m.set(b, a, m.at(a, b));
                }
            }
        }
        for jn in 0..self.tree.n_q() {
            for ex in &self.tree.extras {
                if ex.name == self.tree.q_names[jn] {
                    m.set(6 + jn, 6 + jn, m.at(6 + jn, 6 + jn) + ex.armature);
                }
            }
        }
    }

    /// inverse_dynamics: tau = M alpha + C nu + g in one Newton-Euler pass; nu = [omega, v, qd],
    /// alpha = [omega_dot, v_dot, qdd]. Base rows are the world wrench at the root origin, joint rows
    /// the actuator torques, armature on the diagonal.
    pub fn inverse_dynamics(
        &mut self,
        base_p: Vec3,
        base_q: Quat,
        q: &[f64],
        nu: &[f64],
        alpha: &[f64],
    ) -> Vec<f64> {
        let mut out = Vec::new();
        self.inverse_dynamics_into(base_p, base_q, q, nu, alpha, &mut out);
        out
    }

    /// inverse_dynamics_into is inverse_dynamics into the caller's vector, through this model's scratch.
    pub fn inverse_dynamics_into(
        &mut self,
        base_p: Vec3,
        base_q: Quat,
        q: &[f64],
        nu: &[f64],
        alpha: &[f64],
        out: &mut Vec<f64>,
    ) {
        self.frames(base_p, base_q, q);
        let nn = self.tree.nodes.len();
        // outward pass: per-node world omega/alpha and frame-origin linear acceleration, the +9.81
        // pseudo-acceleration entering at the root.
        self.sc_om.clear();
        self.sc_om.resize(nn, Vec3::default());
        self.sc_al.clear();
        self.sc_al.resize(nn, Vec3::default());
        self.sc_ao.clear();
        self.sc_ao.resize(nn, Vec3::default());
        let (om, al, ao) = (&mut self.sc_om, &mut self.sc_al, &mut self.sc_ao);
        for (i, nd) in self.tree.nodes.iter().enumerate() {
            if nd.parent < 0 {
                om[i] = Vec3::new(nu[0], nu[1], nu[2]);
                al[i] = Vec3::new(alpha[0], alpha[1], alpha[2]);
                ao[i] = Vec3::new(alpha[3], alpha[4], alpha[5] + 9.81);
            } else {
                let p = nd.parent as usize;
                let z = self.tree.world_z(&self.fr_r, i);
                let qd = nu[6 + nd.jo as usize];
                let qdd = alpha[6 + nd.jo as usize];
                let do_ = self.fr_o[i].sub(self.fr_o[p]);
                ao[i] = ao[p]
                    .add(al[p].cross(do_))
                    .add(om[p].cross(om[p].cross(do_)));
                al[i] = al[p].add(z.scale(qdd)).add(om[p].cross(z.scale(qd)));
                om[i] = om[p].add(z.scale(qd));
            }
        }
        // inward pass: body equation per node, children wrenches summed into the parent.
        self.sc_f.clear();
        self.sc_f.resize(nn, Vec3::ZERO);
        self.sc_n.clear();
        self.sc_n.resize(nn, Vec3::ZERO);
        let (f_out, n_out) = (&mut self.sc_f, &mut self.sc_n);
        let mut i = nn;
        while i > 0 {
            i -= 1;
            let nd = &self.tree.nodes[i];
            let dv = self.fr_r[i].mul_vec3(nd.com);
            let a_c = ao[i].add(al[i].cross(dv)).add(om[i].cross(om[i].cross(dv)));
            let f_c = a_c.scale(nd.mass);
            let h = self.fr_iw[i].mul_vec3(om[i]);
            let om_b = pga_vec_to_biv(om[i].to_array(), [0.0, 0.0, 0.0]);
            let h_b = pga_vec_to_biv(h.to_array(), [0.0, 0.0, 0.0]);
            let ia = self.fr_iw[i].mul_vec3(al[i]);
            let n_b = pga_vec_to_biv(ia.to_array(), [0.0, 0.0, 0.0])
                .add(pga_screw_bracket_angular(om_b, h_b));
            let (n_ax, _) = pga_biv_to_axial(n_b);
            let mut n_i = Vec3::new(n_ax[0], n_ax[1], n_ax[2]).add(dv.cross(f_c));
            let mut f_i = f_c;
            for (k, nd2) in self.tree.nodes.iter().enumerate() {
                if nd2.parent == i as i32 {
                    n_i = n_i
                        .add(n_out[k])
                        .add(self.fr_o[k].sub(self.fr_o[i]).cross(f_out[k]));
                    f_i = f_i.add(f_out[k]);
                }
            }
            f_out[i] = f_i;
            n_out[i] = n_i;
        }
        out.clear();
        out.resize(self.nv, 0.0);
        out[0] = n_out[self.tree.root].x;
        out[1] = n_out[self.tree.root].y;
        out[2] = n_out[self.tree.root].z;
        out[3] = f_out[self.tree.root].x;
        out[4] = f_out[self.tree.root].y;
        out[5] = f_out[self.tree.root].z;
        for (i, nd) in self.tree.nodes.iter().enumerate() {
            if nd.jo >= 0 {
                let jo = nd.jo as usize;
                out[6 + jo] = self.tree.world_z(&self.fr_r, i).dot(n_out[i]);
                for ex in &self.tree.extras {
                    if ex.name == nd.joint {
                        out[6 + jo] += ex.armature * alpha[6 + jo];
                    }
                }
            }
        }
    }

    /// gravity_torques: the gravity-compensation vector ID(q, 0, 0).
    pub fn gravity_torques(&mut self, base_p: Vec3, base_q: Quat, q: &[f64]) -> Vec<f64> {
        let mut out = Vec::new();
        self.gravity_torques_into(base_p, base_q, q, &mut out);
        out
    }

    pub fn gravity_torques_into(
        &mut self,
        base_p: Vec3,
        base_q: Quat,
        q: &[f64],
        out: &mut Vec<f64>,
    ) {
        let mut zero = std::mem::take(&mut self.sc_zero);
        zero.clear();
        zero.resize(self.nv, 0.0);
        self.inverse_dynamics_into(base_p, base_q, q, &zero, &zero, out);
        self.sc_zero = zero;
    }

    /// bias_torques: C nu alone = ID(q, nu, 0) - ID(q, 0, 0).
    pub fn bias_torques(&mut self, base_p: Vec3, base_q: Quat, q: &[f64], nu: &[f64]) -> Vec<f64> {
        let mut out = Vec::new();
        self.bias_torques_into(base_p, base_q, q, nu, &mut out);
        out
    }

    /// bias_torques_into is bias_torques into the caller's vector (two inverse passes).
    pub fn bias_torques_into(
        &mut self,
        base_p: Vec3,
        base_q: Quat,
        q: &[f64],
        nu: &[f64],
        out: &mut Vec<f64>,
    ) {
        let mut zero = std::mem::take(&mut self.sc_zero);
        zero.clear();
        zero.resize(self.nv, 0.0);
        let mut id = std::mem::take(&mut self.sc_id);
        self.inverse_dynamics_into(base_p, base_q, q, nu, &zero, &mut id);
        self.gravity_torques_into(base_p, base_q, q, out);
        for i in 0..self.nv {
            out[i] = id[i] - out[i];
        }
        self.sc_zero = zero;
        self.sc_id = id;
    }
}

/// Clone REBUILDS the frame cache rather than copying it (see `clone_shallow`).
impl Clone for TreeDynamicsModel {
    fn clone(&self) -> TreeDynamicsModel {
        self.clone_shallow()
    }
}
