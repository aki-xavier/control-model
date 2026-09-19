// pga_dynamics.rs — GA-dynamics on P(R*_{3,0,1}), same layer as pga_layer.rs, and the sole in-loop dynamics
// backend for the fixed-base arm: ID(q,qd,0) = C·qd + g and ID(q,0,qdd) = M·qdd + g at machine precision,
// payload included (tests/pga_dynamics.rs). The geometry (frames, world axes, world inertia maps) is PGA —
// versor conjugation of the COM-frame tensor — and the body equation uses the screw coadjoint action.

use crate::pga_layer::{euc_part, pga_biv_to_axial, pga_vec_to_biv, vec3_from_pga_vec};
use crate::urdf::UrdfChain;
use control_math::mat::Mat;
use control_math::vec3::Vec3;
use pga::Multivector;

/// pga_cross_biv_vec: cross(omega_axial, v) from grade-1 of b*v, negated (grade-1 is -(a x v) in this basis).
pub fn pga_cross_biv_vec(b: Multivector, v: Vec3) -> Vec3 {
    vec3_from_pga_vec(b.gp(pga::mv_vector(v.x, v.y, v.z, 0.0)).grade(1)).scale(-1.0)
}

/// pga_screw_bracket_angular: angular part of the coadjoint action ad*_a(b) for a = (wa, va) and b = (h, p):
/// wa x h + va x p; the raw commutator [A,B]/2 carries a sign flip under this crate's I-dual embedding, hence the negation.
pub fn pga_screw_bracket_angular(a: Multivector, b: Multivector) -> Multivector {
    let (_, va) = pga_biv_to_axial(a);
    let (_, vb) = pga_biv_to_axial(b);
    let comm = euc_part(a)
        .gp(euc_part(b))
        .sub(euc_part(b).gp(euc_part(a)))
        .scale(-0.5);
    let vp = Vec3::new(va[0], va[1], va[2]).cross(Vec3::new(vb[0], vb[1], vb[2]));
    comm.add(pga_vec_to_biv([vp.x, vp.y, vp.z], [0.0, 0.0, 0.0]))
}

/// PgaDynamicsModel: the same closed-form formulas as the E3 layer, with motors and PGA screw bivectors.
pub struct PgaDynamicsModel {
    pub n: usize,
    pub chain: UrdfChain,
    /// payload: point mass attached at the terminal link (com_offset expressed in the terminal link frame).
    pub payload_mass: f64,
    pub payload_com: Vec3,
    // q-keyed frame cache (FK + per-link world axes and inertias), shared by all terms in a tick; pub
    // because the plant layer of the crate that consumes this one reads the same frames — callers must
    // not mutate what they read.
    pub fr_q: Vec<f64>,
    pub fr_o: Vec<Vec3>,
    pub fr_r: Vec<Mat>,
    pub fr_z: Vec<Vec3>,
    pub fr_iw: Vec<Mat>,
}

impl PgaDynamicsModel {
    pub fn new(chain: UrdfChain) -> PgaDynamicsModel {
        PgaDynamicsModel {
            n: chain.n,
            chain,
            payload_mass: 0.0,
            payload_com: Vec3::default(),
            fr_q: Vec::new(),
            fr_o: Vec::new(),
            fr_r: Vec::new(),
            fr_z: Vec::new(),
            fr_iw: Vec::new(),
        }
    }

    /// invalidate_frames drops the q-keyed frame cache. Only the world inertia map carries a parameter
    /// (link_ic, through scale_inertia), but every mutator drops the whole cache rather than deciding for itself.
    fn invalidate_frames(&mut self) {
        self.fr_q = Vec::new();
    }

    /// scale_mass: model-mismatch knob — multiply every link mass by s and the payload point mass too.
    pub fn scale_mass(&mut self, s: f64) {
        for i in 0..self.chain.link_mass.len() {
            self.chain.link_mass[i] *= s;
        }
        if self.payload_mass > 0.0 {
            self.payload_mass *= s;
        }
        self.invalidate_frames();
    }

    /// scale_inertia: model-mismatch knob — multiply every link inertia tensor.
    pub fn scale_inertia(&mut self, s: f64) {
        for i in 0..self.chain.link_ic.len() {
            let mut m = self.chain.link_ic[i].clone();
            for r in 0..m.rows {
                for c in 0..m.cols {
                    m.set(r, c, m.at(r, c) * s);
                }
            }
            self.chain.link_ic[i] = m;
        }
        // without this the cached world inertia maps survive the change, and a mismatch applied between two
        // evaluations of one pose takes effect in the masses and not in the inertias until the next q change
        self.invalidate_frames();
    }

    /// offset_com: model-mismatch knob — shift every link COM by (frame-relative) dv, a constant offset in the link frame.
    pub fn offset_com(&mut self, dv: Vec3) {
        for i in 0..self.chain.link_com.len() {
            self.chain.link_com[i] = self.chain.link_com[i].add(dv);
        }
        self.invalidate_frames();
    }

    pub fn frames(&mut self, q: &[f64]) {
        let mut same = self.fr_q.len() == self.n;
        if same {
            for i in 0..self.n {
                if self.fr_q[i] != q[i] {
                    same = false;
                    break;
                }
            }
        }
        if same {
            return;
        }
        self.fr_q = q.to_vec();
        let (o, r) = self.chain.fk(q);
        self.fr_o = o;
        self.fr_r = r;
        self.fr_z = vec![Vec3::default(); self.n];
        self.fr_iw = vec![Mat::zeros(0, 0); self.n];
        for i in 0..self.n {
            self.fr_z[i] = self.chain.world_z(&self.fr_r, i);
            self.fr_iw[i] = pga_world_inertia(&self.fr_r[i], &self.chain.link_ic[i]);
        }
    }

    fn link_com_w(&self, ll: usize) -> Vec3 {
        self.fr_o[ll].add(self.fr_r[ll].mul_vec3(self.chain.link_com[ll]))
    }

    fn payload_w(&self) -> Vec3 {
        let n = self.n;
        self.link_com_w(n - 1)
            .add(self.fr_r[n - 1].mul_vec3(self.payload_com))
    }

    /// mass_matrix: M_ij = sum_{L >= max(i,j)} m vbar_i . vbar_j + (I_w z_i).z_j, symmetric-filled, all geometry from the cached PGA frames.
    pub fn mass_matrix(&mut self, q: &[f64]) -> Mat {
        self.frames(q);
        let n = self.n;
        let mut cs = vec![Vec3::default(); n];
        for (ll, c) in cs.iter_mut().enumerate() {
            *c = self.link_com_w(ll);
        }
        let has_pl = self.payload_mass > 0.0;
        let mut pp = Vec3::new(0.0, 0.0, 0.0);
        if has_pl {
            pp = self.payload_w();
        }
        let mut m = Mat::zeros(n, n);
        for i in 0..n {
            let zi = self.fr_z[i];
            for j in i..n {
                let zj = self.fr_z[j];
                let mut s = 0.0;
                for ll in 0..n {
                    if j > ll {
                        continue; // j >= i, so both joints move link ll iff j <= ll
                    }
                    let vi = zi.cross(cs[ll].sub(self.fr_o[i]));
                    let vj = zj.cross(cs[ll].sub(self.fr_o[j]));
                    s +=
                        self.chain.link_mass[ll] * vi.dot(vj) + self.fr_iw[ll].mul_vec3(zi).dot(zj);
                }
                if has_pl {
                    let vi = zi.cross(pp.sub(self.fr_o[i]));
                    let vj = zj.cross(pp.sub(self.fr_o[j]));
                    s += self.payload_mass * vi.dot(vj);
                }
                m.set(i, j, s);
                m.set(j, i, s);
            }
        }
        m
    }

    /// gravity_torques: rnea compensation convention (a0 = +g pseudo).
    pub fn gravity_torques(&mut self, q: &[f64]) -> Vec<f64> {
        self.frames(q);
        let g = Vec3::new(0.0, 0.0, 9.81);
        let has_pl = self.payload_mass > 0.0;
        let mut pp = Vec3::new(0.0, 0.0, 0.0);
        if has_pl {
            pp = self.payload_w();
        }
        let mut out = vec![0.0; self.n];
        for i in 0..self.n {
            let z = self.fr_z[i];
            let mut s = 0.0;
            for ll in i..self.n {
                let c = self.link_com_w(ll);
                s += self.chain.link_mass[ll] * z.cross(c.sub(self.fr_o[i])).dot(g);
            }
            if has_pl {
                s += self.payload_mass * z.cross(pp.sub(self.fr_o[i])).dot(g);
            }
            out[i] = s;
        }
        out
    }

    /// inverse_dynamics: tau = M qdd + C qd + g in a single Newton-Euler pass — the body equation F = I Vdot + ad*_V(I V) on PGA world inertias, gravity through a0 = +g, payload included.
    pub fn inverse_dynamics(&mut self, q: &[f64], qd: &[f64], qdd: &[f64]) -> Vec<f64> {
        self.frames(q);
        let n = self.n;
        let mut omega = Vec3::new(0.0, 0.0, 0.0);
        let mut alpha = Vec3::new(0.0, 0.0, 0.0);
        let mut a_o = Vec3::new(0.0, 0.0, 9.81);
        let mut om_link = vec![Vec3::default(); n];
        let mut al_link = vec![Vec3::default(); n];
        let mut a_link = vec![Vec3::default(); n];
        for i in 0..n {
            let z = self.fr_z[i];
            // origin acceleration of frame i: d_i = o_i - o_{i-1} is rigid in link i-1, so propagate with the PREVIOUS link's omega/alpha, then advance
            let do_ = self.fr_o[i].sub(if i == 0 {
                Vec3::new(0.0, 0.0, 0.0)
            } else {
                self.fr_o[i - 1]
            });
            a_o = a_o.add(alpha.cross(do_)).add(omega.cross(omega.cross(do_)));
            alpha = alpha.add(z.scale(qdd[i])).add(omega.cross(z.scale(qd[i])));
            omega = omega.add(z.scale(qd[i]));
            om_link[i] = omega;
            al_link[i] = alpha;
            a_link[i] = a_o;
        }
        let mut f_out = vec![Vec3::default(); n];
        let mut n_out = vec![Vec3::default(); n];
        let mut i = n;
        while i > 0 {
            i -= 1;
            let dv = self.fr_r[i].mul_vec3(self.chain.link_com[i]);
            let a_c = a_link[i]
                .add(al_link[i].cross(dv))
                .add(om_link[i].cross(om_link[i].cross(dv)));
            let f_c = a_c.scale(self.chain.link_mass[i]);
            let h = self.fr_iw[i].mul_vec3(om_link[i]);
            let om_b = pga_vec_to_biv(om_link[i].to_array(), [0.0, 0.0, 0.0]);
            let h_b = pga_vec_to_biv(h.to_array(), [0.0, 0.0, 0.0]);
            let ia = self.fr_iw[i].mul_vec3(al_link[i]);
            let n_b = pga_vec_to_biv(ia.to_array(), [0.0, 0.0, 0.0])
                .add(pga_screw_bracket_angular(om_b, h_b));
            let (n_ax, _) = pga_biv_to_axial(n_b);
            let nc = Vec3::new(n_ax[0], n_ax[1], n_ax[2]);
            let mut n_i = nc.add(dv.cross(f_c));
            let mut f_i = f_c;
            if i == n - 1 && self.payload_mass > 0.0 {
                let p_p = self.payload_w();
                let d_p = p_p.sub(self.fr_o[i]);
                let a_p = a_link[i]
                    .add(al_link[i].cross(d_p))
                    .add(om_link[i].cross(om_link[i].cross(d_p)));
                let f_p = a_p.scale(self.payload_mass);
                f_i = f_i.add(f_p);
                n_i = n_i.add(d_p.cross(f_p));
            }
            if i < n - 1 {
                n_i = n_i
                    .add(n_out[i + 1])
                    .add(self.fr_o[i + 1].sub(self.fr_o[i]).cross(f_out[i + 1]));
                f_i = f_i.add(f_out[i + 1]);
            }
            f_out[i] = f_i;
            n_out[i] = n_i;
        }
        let mut out = vec![0.0; n];
        for i in 0..n {
            out[i] = self.fr_z[i].dot(n_out[i]);
        }
        out
    }

    /// bias_torques: C(q,qd)·qd alone = ID(q, qd, 0) - gravity (payload-consistent).
    pub fn bias_torques(&mut self, q: &[f64], qd: &[f64]) -> Vec<f64> {
        let id = self.inverse_dynamics(q, qd, &vec![0.0; self.n]);
        let g = self.gravity_torques(q);
        let mut out = vec![0.0; self.n];
        for i in 0..self.n {
            out[i] = id[i] - g[i];
        }
        out
    }
}

/// pga_world_inertia: world-frame inertia map I_w = R Ic R^T of a link as a 3x3 matrix — the rotor conjugation of the COM-frame tensor done as plain 3x3 arithmetic; the module's central geometry claim, checked by tests/pga_dynamics.rs.
pub fn pga_world_inertia(r: &Mat, ic: &Mat) -> Mat {
    r.mul(ic).mul(&r.transposed())
}
