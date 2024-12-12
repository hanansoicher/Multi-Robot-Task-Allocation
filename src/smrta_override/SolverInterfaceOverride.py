import time
import math
from enum import Enum
import z3

solver = None
solver_list = ['z3']
def is_z3(): return solver == 'z3'

class Result(Enum):
    sat = 1
    unsat = 2
    unknown = 3

class SolverInterface:
    def __init__(self, solver_name, theory):
        global solver
        # assert solver_name in solver_list, f"Solver not available ({solver})"
        solver = 'z3'
        self.theory = theory
        self.n_scopes = 0
        self.s = None
        if is_z3():
            self.s = z3.Solver()

    def add(self, term):
        if type(term) is Bool:
            term = term.var
        elif type(term) is list:
            assert all(type(t) is Bool for t in term)
            term = [t.var for t in term]
            
        if is_z3():
            self.s.add(term)

    def assertions(self):
        if is_z3():
            assertions = self.s.assertions()
        else: raise NotImplementedError
        return [Bool(var=a) for a in assertions]

    def check(self, assumptions=None, timeout=None):
        # timeout in secs
        if assumptions is not None:
            if type(assumptions) is Bool: 
                assumptions = [assumptions]
            assert all(type(a) is Bool for a in assumptions)

        if is_z3():
            if timeout is not None:
                self.s.set(timeout=int(timeout * 1e3))

            if assumptions is None: assumptions = []
            assumptions = [a.var for a in assumptions]
            res = self.s.check(assumptions)

        if res == z3.sat:
            if is_z3():
                self.z3_model = self.s.model()
            self.res = Result.sat
        elif res == z3.unsat:
            self.res = Result.unsat
        elif res == z3.unknown:
            self.res = Result.unknown
        return self.res

    def model(self):
        if is_z3():
            assert self.res == z3.sat
            return self.s.model()
        else: raise NotImplementedError

    def get_value(self, var):
        assert self.res == Result.sat, f"No model available: result is `{self.res}`"
        if is_z3():
            if self.theory in ['QF_UFBV', 'QF_UFLIA']:
                x = self.z3_model[var.var]
                if type(var.var) in [z3.ArithRef, z3.BitVecRef]:
                    return x.as_long()
                else: return x
        else: raise NotImplementedError

    def to_smt2(self):
        if is_z3():
            return f'(set-logic {self.theory})\n' + self.s.to_smt2() + '(get-model)\n(exit)\n'
        else: raise NotImplementedError

    def push(self):
        self.s.push()
        self.n_scopes += 1

    def pop(self):
        self.s.pop()
        self.n_scopes -= 1

    def num_scopes(self):
        return self.n_scopes

    def reset(self):
        if is_z3():
            self.s.reset()
        else: raise NotImplementedError
        self.n_scopes = 0

def check_enough_bits(x, size):
    return (-1 << (size-1)) <= x < (1 << (size-1))

def get_z3_term(that):
    if type(that) in [Bool, Int, BitVec]: return that.var
    elif type(that) is int or type(that) is bool: return that
    else: raise NotImplementedError

class Bool:
    def __init__(self, name=None, var=None):
        self.name = name
        if var is not None:
            self.var = var
        elif is_z3():
            self.var = z3.Bool(name)
        else: raise NotImplementedError

    def __str__(self): return str(self.var)

    def __repr__(self): return str(self)

class Int:
    def __init__(self, name=None, var=None):
        global solver
        assert solver in solver_list, f"Solver not available ({solver})"
        self.name = name
        if var is not None:
            if is_z3():
                self.var = var
        elif is_z3():
            self.var = z3.Int(name)
        else: raise NotImplementedError

    def __str__(self): return self.name

    def __repr__(self): return str(self)

    def __add__(self, that):
        if is_z3():
            var = self.var + get_z3_term(that)
        else: raise NotImplementedError
        return Int(var=var)

    def __sub__(self, that):
        if is_z3():
            var = self.var - get_z3_term(that)
        else: raise NotImplementedError
        return Int(var=var)

    def __mul__(self, that):
        assert type(that) is int
        if is_z3():
            var = self.var * that
        else: raise NotImplementedError
        return Int(var=var)

    def __mod__(self, that):
        assert type(that) is int
        if is_z3():
            var = self.var % that
        else: raise NotImplementedError
        return Int(var=var)

    def __eq__(self, that):
        if is_z3():
            term = self.var == get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

    def __ne__(self, that):
        if is_z3():
            term = self.var != get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

    def __ge__(self, that):
        if is_z3():
            term = self.var >= get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

    def __le__(self, that):
        if is_z3():
            term = self.var <= get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

    def __gt__(self, that):
        if is_z3():
            term = self.var > get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

    def __lt__(self, that):
        if is_z3():
            term = self.var < get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

class BitVec:
    def __init__(self, name=None, num_bits=None, var=None):
        global solver
        assert solver in solver_list, f"Solver not available ({solver})"
        self.name = name
        self.size = num_bits
        if var is not None:
            self.var = var
            if is_z3():
                self.size = None
        elif is_z3():
            self.var = z3.BitVec(name, num_bits)
        else: raise NotImplementedError

    def __str__(self): return self.name

    def __repr__(self): return str(self)

    def __add__(self, that):
        if is_z3():
            var = self.var + get_z3_term(that)
        else: raise NotImplementedError
        return BitVec(var=var)

    def __sub__(self, that):
        if is_z3():
            var = self.var - get_z3_term(that)
        else: raise NotImplementedError
        return BitVec(var=var)
    
    def __and__(self, that):
        if is_z3():
            var = self.var & get_z3_term(that)
        else: raise NotImplementedError
        return BitVec(var=var)

    def __or__(self, that):
        if is_z3():
            var = self.var | get_z3_term(that)
        else: raise NotImplementedError
        return BitVec(var=var)

    def __eq__(self, that):
        if is_z3():
            term = self.var == get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

    def __ne__(self, that):
        if is_z3():
            term = self.var != get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

    def __ge__(self, that):
        if is_z3():
            term = self.var >= get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

    def __le__(self, that):
        if is_z3():
            term = self.var <= get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

    def __gt__(self, that):
        if is_z3():
            term = self.var > get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

    def __lt__(self, that):
        if is_z3():
            term = self.var < get_z3_term(that)
        else: raise NotImplementedError
        return Bool(var=term)

def BitVecVal(x, num_bits):
    if is_z3():
        var = z3.BitVecVal(x, num_bits)
    else: raise NotImplementedError
    return BitVec(var=var)

class BitVecSort:
    def __init__(self, num_bits):
        self.size = num_bits
        if is_z3():
            self.sort = z3.BitVecSort(num_bits)
        else: raise NotImplementedError

class Function:
    def __init__(self, name, *args):
        assert len(args) >= 2
        self.domain = args[:-1]
        sorts = [arg.sort for arg in args]
        if is_z3():
            self.var = z3.Function(name, *sorts)
        else: raise NotImplementedError
        
    def __call__(self, *args):
        assert len(args) == len(self.domain)
        if is_z3():
            solver_args = [get_z3_term(arg) for arg in args]
            var = self.var(*solver_args)
        else: raise NotImplementedError
        return BitVec(var=var)


# Boolean operators
def Not(t):
    assert type(t) is Bool
    if is_z3():
        term = z3.Not(get_z3_term(t))
    else: raise NotImplementedError
    return Bool(var=term)

def And(*args):
    if len(args) == 0 or (len(args) == 1 and len(args[0]) == 0): # Empty
        if is_z3(): term = True
        else: raise NotImplementedError
    else:
        if len(args) == 1 and type(args[0]) is list:
            args = args[0]

        if is_z3():
            args = [get_z3_term(arg) for arg in args]
            term = z3.And(args) if len(args) > 1 else args[0]
        else: raise NotImplementedError
    return Bool(var=term)

def Or(*args):
    if len(args) == 0 or (len(args) == 1 and len(args[0]) == 0): # Empty
        if is_z3(): term = False
        else: raise NotImplementedError
    else:
        if len(args) == 1 and type(args[0]) is list:
            args = args[0]

        if is_z3():
            args = [get_z3_term(arg) for arg in args]
            term = z3.Or(args) if len(args) > 1 else args[0]
        else: raise NotImplementedError
    return Bool(var=term)

def Implies(t1, t2):
    if is_z3():
        term = z3.Implies(get_z3_term(t1), get_z3_term(t2))
    else: raise NotImplementedError
    return Bool(var=term)

def If(t1, t2, t3):
    if is_z3():
        term = z3.If(get_z3_term(t1), get_z3_term(t2), get_z3_term(t3))
    else: raise NotImplementedError
    return BitVec(var=term)

if __name__ == '__main__':
    import IPython; IPython.embed()
