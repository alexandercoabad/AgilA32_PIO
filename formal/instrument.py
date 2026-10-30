#!/usr/bin/env python3
"""instrument.py <design.v> <props.vh> <out.v>
Copies <design.v> to <out.v> with `include "<props.vh>" (inside `ifdef FORMAL) inserted just before the last `endmodule`, so the
formal properties can see the module's internal registers.  The design file itself is never modified."""
import sys, os
src, props, out = sys.argv[1:4]
s = open(src).read()
i = s.rindex("endmodule")
inc = '\n`ifdef FORMAL\n`include "%s"\n`endif\n' % os.path.basename(props)
open(out, "w").write(s[:i] + inc + s[i:])
