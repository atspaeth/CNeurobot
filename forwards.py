import numpy as np

try:
    import cpgcompiler as cpg
except ImportError:
    from . import cpgcompiler as cpg

class SingleCPG(cpg.CPGBase):
    def __init__(self, Gexc=20, Ginh=60, Gffw=10, Gfb=8, Gslow=3, Gmusc=1):
        
        G = cpg.connectivity(
            cpg.module(0, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module(3, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module(6, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module(9, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module_loop(0,3,6,9, Gfb=Gfb, Gffw=Gffw) + [
                # Excitatory connections to muscle cells.
                (1,12, Gmusc),
                (4,13, Gmusc),
                (7,14, Gmusc),
                (10,15, Gmusc)
            ], N=16)

        is_excitatory = np.array([True, True, False]*4 + [True]*4)
        super().__init__(G, is_excitatory, 12)
        
    def muscle_activations(self):
        return np.clip(self.V[-4:] - np.roll(self.V[-4:], 2), -1, 1)

class SingleFeedbackCPG(SingleCPG):
    def propriocept(self, pos):
        Iout = np.zeros(12)
        errs = 1 - np.roll(pos, 1) + np.roll(pos, -1)
        Iout[0::3] = -25 *errs
        return Iout


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
            description='Generate C code for a forward CPG.')
    parser.add_argument('file', help='output filename')
    args = parser.parse_args()

    with open(args.file, 'w') as f:
        SingleCPG().dump_source(f)
