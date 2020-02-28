import numpy as np
from braingeneers.drylab import Organoid, NEURON_TYPES

def connectivity(jig, N=None):
    """
    Create the connectivity matrix given a list of connections in the
    form of tuples (j,i,g) which indicate a synapse of conductance g
    from cell j to cell i. It's optional but a good idea to also pass
    the number of cells N. 
    """
    if N is None:
        N = max(max(i,j) for j,i,_ in jig) + 1
    
    G = np.zeros((N,N))
    for j,i,gij in jig:
        G[i,j] += gij
    return G


def module(i, *, Gexc, Ginh, Gslow):
    """
    Connectivity for a neural oscillator at index i and i+1,
    plus a kill-switch inhibitory neuron at index i+2.
    """
    return [
        # Mutually excitatory loop.
        ( i,  i+1,  Gexc), 
        (i+1,  i,   Gexc),
        # "Kill switch".
        (i+2,  i,  Ginh),
        (i+2, i+1, Ginh),
        # Lower the kill switch threshold as long as the
        # module continues to be active.
        ( i,  i+2, Gslow),
        (i+1, i+2, Gslow)
    ]


def module_loop(*idces, Gfb, Gffw):
    """
    Make a loop from the oscillator modules at the provided 
    indices. The second cell in each module is connected to 
    the first cell in the next, and the first cell in each 
    module is connected to the third cell in the previous.
    """
    cellA = np.array(idces)
    cellB = np.array(idces) + 1
    kill = np.array(idces) + 2
    return [
        (i,j, Gffw) for i,j in zip(cellB, np.roll(cellA, -1))
    ] + [
        (i,j, Gfb) for i,j in zip(cellA, np.roll(kill, 1))
    ]


class CPGBase(Organoid):
    def __init__(self, G, is_excitatory, n_neurons):

        self.cell_types = ['rs' if exc else 'lts' 
                           for exc in is_excitatory]

        a, b, c, d, C, k, Vr, Vt, Vp, Vn, tau = \
                np.where(is_excitatory[:,None], 
                        NEURON_TYPES['rs'], 
                        NEURON_TYPES['lts']).T

        super().__init__(G=G, tau=tau, XY=None, a=a, b=b, c=c, d=d,
                         C=C, k=k, Vr=Vr, Vt=Vt, Vp=Vp, Vn=Vn)
        
        self.n_neurons = n_neurons
        self.n_muscles = self.N - n_neurons
        
        self.start()
        
    def step(self, *args, dt, pos):
        Iin = np.hstack((self.propriocept(pos), np.zeros(self.n_muscles)))
        super().step(dt=dt, Iin=Iin)
        
    def propriocept(self, pos):
        """
        Returns a current to send to each neuron indicating
        how far away its predecessor's actuators are from 
        the desired target position. 
        """
        return np.zeros(self.n_neurons)
        
    def muscle_activations(self):
        pass
        
    def start(self):
        self.fired[0] = True

    def dump_source(self):
        """
        Prints the connectivity and parameters as C source code.
        """
        print('#include "libneurobot.h"\n')
        print(f'#define N_CELLS {self.N}\n')

        print('struct state states[N_CELLS] = {')
        print('  [0 ... N_CELLS-1] = {.v=-60, .u=0, .i=0, .j=0}')
        print('};\n')

        for type in set(self.cell_types):
            a, b, c, d, C, k, vr, vt, vp, vn, tau = NEURON_TYPES[type]
            print(f'const struct params {type.upper()} = {{')
            print(f'  .a={a}, .b={b}, .c={c}, .d={d},')
            print(f'  .C={C}, .k={k}, .tau={tau},')
            print(f'  .vr={vr}, .vt={vt}, .vp={vp}, .vn={vn}')
            print(f'}};\n')

        print('\nconst struct params *params[N_CELLS] = {')
        for i,type in enumerate(self.cell_types):
            print(f'  [{i}] = &{type.upper()},')
        print('};\n')

        print('const float G[N_CELLS][N_CELLS] = {')
        for (i,j),gij in np.ndenumerate(self.G):
            if gij != 0:
                print(f'\t[{i}][{j}] = {gij},')
        print(f'}};')
        
             
class SingleCPG(CPGBase):
    def __init__(self, Gexc=20, Ginh=40, Gffw=10, Gfb=8, Gslow=3, Gmusc=1):
        
        G = connectivity(
            module(0, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module(3, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module(6, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module(9, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module_loop(0,3,6,9, Gfb=Gfb, Gffw=Gffw) + [
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


class DoubleCPG(CPGBase):
    def __init__(self, Gexc=1000, Ginh=-1000, 
            Gffw=400, Gfb=400, Gslow=100, Gmusc=40):
        G = connectivity(
            module(0, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module(3, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module(6, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module(9, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module(12, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module(15, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module(18, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module(21, Gexc=Gexc, Ginh=Ginh, Gslow=Gslow) + 
            module_loop(0,3,6,9, Gfb=Gfb, Gffw=Gffw) + 
            module_loop(21,18,15,12, Gfb=Gfb, Gffw=Gffw) + [
                (1,24, Gmusc), (13,24, Gmusc),
                (4,25, Gmusc), (16,25, Gmusc),
                (7,26, Gmusc), (19,26, Gmusc),
                (10,27, Gmusc), (22,27, Gmusc)
            ], N=28)
    
        super().__init__(G, 24)
        
    def muscle_activations(self):
        return self.V[-4:] - np.roll(self.V[-4:], 2)


class DoubleFeedbackCPG(DoubleCPG):
    def propriocept(self, pos):
        Iout = np.zeros(self.n_neurons)
        errs = 1 - np.roll(pos, 1) + np.roll(pos, -1)
        Ioutjkk[:12:3] = -5 * errs

        errs = 1 - np.roll(pos, -1) + np.roll(pos, 1)
        Iout[12::3] = -5 * errs
        return Iout

if __name__ == '__main__':
    import sys
    for arg in sys.argv[1:]:
        cpg = eval(arg)
    cpg.dump_source()
