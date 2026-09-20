# control-model — the model basis: the XML/URDF/MJCF readers, the chains and their PGA kinematics and dynamics.
#
#   make test          # this crate's suite, then the comment rules over the tree
#   make comments      # the comment rules alone, with the local approximation for the rest
#
# The comment rules are a dev-dependency of this crate: they read text, and nothing here links them.
#
# Every Cargo command below is run as `mbx <subcommand>` (the build-cache wrapper). The rules live in
# ../comment-why, read text, and need no toolchain of their own: the gate inside `mbx test` is
# tests/comment_why.rs, and `make comments` is the same rules over the working tree, with that crate's
# local approximation for the comments the rules cannot decide.

COMMENT_WHY ?= ../comment-why

.PHONY: test comments

test:
	mbx test
	$(MAKE) comments

comments:
	mbx run --quiet --manifest-path $(COMMENT_WHY)/Cargo.toml --bin comment-why -- --review
