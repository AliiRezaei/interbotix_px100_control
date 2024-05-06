from sympy import zeros, eye, Matrix, sign
from copy import deepcopy


class RicursiveNewtonEuler():

    def __init__(self):
        self.identity = lambda x: x
        self._frictionterms = set(['Coulomb', 'viscous', 'offset'])

    def forward(self, rbtdef, geom, ifunc=None):
        if not ifunc:
            ifunc = self.identity

        w  = list(range(0, rbtdef.dof + 1))
        dw = list(range(0, rbtdef.dof + 1))
        dV = list(range(0, rbtdef.dof + 1))
        U  = list(range(0, rbtdef.dof + 1))

        w[-1]  = zeros(3, 1)
        dw[-1] = zeros(3, 1)
        dV[-1] = -rbtdef.gravityacc
        U[-1]  = zeros(3, 3)

        z = Matrix([0, 0, 1])

        # Forward
        for i in range(rbtdef.dof):

            s = rbtdef._links_sigma[i]
            ns = 1 - s

            w_pj = geom.Rdh[i].T * w[i - 1]

            w[i] = w_pj + ns * rbtdef.dq[i] * z
            w[i] = ifunc(w[i])

            dw[i] = geom.Rdh[i].T * dw[i - 1] + ns * \
                (rbtdef.ddq[i] * z + w_pj.cross(rbtdef.dq[i] * z).reshape(3, 1))
            dw[i] = ifunc(dw[i])

            dV[i] = geom.Rdh[i].T * (dV[i - 1] + U[i - 1] * geom.pdh[i]) + s * (
                rbtdef.ddq[i] * z + 2 * w_pj.cross(rbtdef.dq[i] * z).reshape(3, 1))
            dV[i] = ifunc(dV[i])

            U[i] = self.skew(dw[i]) + self.skew(w[i]) ** 2
            U[i] = ifunc(U[i])

        return w, dw, dV, U
    
    def backward(self, rbtdef, geom, fw_results, ifunc=None):

        w, dw, dV, U = fw_results

        if not ifunc:
            ifunc = self.identity

        # extend Rdh so that Rdh[dof] return identity
        Rdh = geom.Rdh + [eye(3)]
        # extend pdh so that pRdh[dof] return zero
        pdh = geom.pdh + [zeros(3, 1)]

        F = list(range(rbtdef.dof))
        M = list(range(rbtdef.dof))
        f = list(range(rbtdef.dof + 1))
        m = list(range(rbtdef.dof + 1))

        f[rbtdef.dof] = zeros(3, 1)
        m[rbtdef.dof] = zeros(3, 1)

        z = Matrix([0, 0, 1])

        tau = zeros(rbtdef.dof, 1)

        fric = self.frictionforce(rbtdef)
        Idrive = self.driveinertiaterm(rbtdef)

        # Backward
        for i in range(rbtdef.dof - 1, -1, -1):

            s = rbtdef._links_sigma[i]
            ns = 1 - s

            F[i] = rbtdef.m[i] * dV[i] + U[i] * Matrix(rbtdef.l[i])
            F[i] = ifunc(F[i])

            M[i] = rbtdef.L[i] * dw[i] + w[i].cross(
                rbtdef.L[i] * w[i]).reshape(3, 1) + \
                Matrix(rbtdef.l[i]).cross(dV[i]).reshape(3, 1)
            M[i] = ifunc(M[i])

            f_nj = Rdh[i + 1] * f[i + 1]

            f[i] = F[i] + f_nj  # + f_e[i]
            f[i] = ifunc(f[i])

            m[i] = M[i] + Rdh[i + 1] * m[i + 1] + \
                pdh[i + 1].cross(f_nj).reshape(3, 1)  # + m_e[i]
            m[i] = ifunc(m[i])

            tau[i] = ifunc(((s * f[i] + ns * m[i]).T * z)[0] + fric[i] + Idrive[i])

        return tau
    
    def frictionforce(self, rbtdef, ifunc=None):
    
        if not ifunc:
            ifunc = self.identity

        fric = zeros(rbtdef.dof, 1)

        if rbtdef.frictionmodel is None or len(rbtdef.frictionmodel) == 0:
            pass
        else:
            askedterms = set(rbtdef.frictionmodel)
            if askedterms.issubset(self._frictionterms):
                if 'viscous' in askedterms:
                    for i in range(rbtdef.dof):
                        fric[i] += rbtdef.fv[i] * rbtdef.dq[i]
                if 'Coulomb' in askedterms:
                    for i in range(rbtdef.dof):
                        fric[i] += rbtdef.fc[i] * sign(rbtdef.dq[i])
                if 'offset' in askedterms:
                    for i in range(rbtdef.dof):
                        fric[i] += rbtdef.fo[i]
                fric[i] = ifunc(fric[i])
            else:
                raise Exception(
                    'Friction model terms \'%s\' not understanded. Use None or a'
                    ' combination of %s.' %
                    (str(askedterms - self._frictionterms), self._frictionterms))

        return fric


    def driveinertiaterm(self, rbtdef, ifunc=None):
        '''Generate drive inertia term (Siplified, neglets gyroscopic effects).'''

        if not ifunc:
            ifunc = self.identity

        driveinertia = zeros(rbtdef.dof, 1)

        if rbtdef.driveinertiamodel is None:
            pass
        elif rbtdef.driveinertiamodel == 'simplified':
            for i in range(rbtdef.dof):
                driveinertia[i] = rbtdef.Ia[i] * rbtdef.ddq[i]
                driveinertia[i] = ifunc(driveinertia[i])
        else:
            raise Exception('Drive inertia model \'%s\' not understanded. Use'
                            ' None or \'simplified\'.' % rbtdef.driveinertiamodel)

        return driveinertia
    
    def skew(self, x):
        return Matrix([[    0,  -x[2],    x[1]],
                       [ x[2],      0,   -x[0]],
                       [-x[1],   x[0],      0]])
    
    def regressor(self, rbtdef, geom, ifunc=None):
    
        if not ifunc:
            ifunc = self.identity

        fw_results = self.forward(rbtdef, geom, ifunc)

        rbtdeftmp = deepcopy(rbtdef)

        dynparms = rbtdef.dynparms()

        Y = zeros(rbtdef.dof, len(dynparms))

        for p, parm in enumerate(dynparms):

            for i in range(rbtdef.dof):
                rbtdeftmp.Le[i] = list(map(
                    lambda x: 1 if x == parm else 0, rbtdef.Le[i]))
                rbtdeftmp.l[i] = Matrix(rbtdef.l[i]).applyfunc(
                    lambda x: 1 if x == parm else 0)

                rbtdeftmp.m[i]  = 1 if rbtdef.m[i]  == parm else 0
                rbtdeftmp.Ia[i] = 1 if rbtdef.Ia[i] == parm else 0
                rbtdeftmp.fv[i] = 1 if rbtdef.fv[i] == parm else 0
                rbtdeftmp.fc[i] = 1 if rbtdef.fc[i] == parm else 0
                rbtdeftmp.fo[i] = 1 if rbtdef.fo[i] == parm else 0

            Y[:, p] = self.backward(rbtdeftmp, geom, fw_results, ifunc=ifunc)

        return Y