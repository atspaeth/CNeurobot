import numpy as np

import cpgcompiler as cpg

class DoubleCPG(cpg.CPGBase):
    def __init__(self, Gexc=20, Ginh=40, Gffw=10, Gfb=8, Gslow=3, Gmusc=1):

        G = cpg.connectivity(
            cpg.module(0, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module(3, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module(6, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module(9, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module(12, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module(15, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module(18, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module(21, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            cpg.module_loop(0,3,6,9, Gfb=Gfb, Gffw=Gffw) + 
            cpg.module_loop(21,18,15,12, Gfb=Gfb, Gffw=Gffw) + [
                (1,24, Gmusc), (13,24, Gmusc),
                (4,25, Gmusc), (16,25, Gmusc),
                (7,26, Gmusc), (19,26, Gmusc),
                (10,27, Gmusc), (22,27, Gmusc)
            ], N=28)
    
        is_excitatory = np.array([True, True, False]*8 + [True]*4)
        super().__init__(G, is_excitatory, 24)
        
    def muscle_activations(self):
        return self.V[-4:] - np.roll(self.V[-4:], 2)


class DoubleFeedbackCPG(DoubleCPG):
    def propriocept(self, pos):
        Iout = np.zeros(self.n_neurons)
        errs = 1 - np.roll(pos, 1) + np.roll(pos, -1)
        Ioutjkk[:12:3] = -25 * errs

        errs = 1 - np.roll(pos, -1) + np.roll(pos, 1)
        Iout[12::3] = -25 * errs
        return Iout


if __name__ == '__main__':
    DoubleCPG().dump_source()
