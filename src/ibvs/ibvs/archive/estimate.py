import numpy as np

class VelocityEstimator():

    def __init__(self):
        """
        Init. velocity estimator, does not begin estimation, but constructs datatypes and stores constants.
        """

        # Gamma gains
        self.y1 = 0.08
        self.y2 = 0.12

        # Update variables
        self.err_f = 0.0
        self.err_0 = None
        self.h = 0.0

        # Anti-peaking values
        self.h1 = 2.7 # Known upper velocity bound
        self.eps0 = 0.2 # Tolerance

        # Velocity estimate
        self.Vp_hat = 0.0

        # Iterations (for estimating elapsed time)
        self.it = 0

    def update_filter(self, dt, err, V_cam, J):
        """
        Updates the filter based on a whole load of different parameters from the IBVS control system.
        """

        self.it += 1

        # Copy initial error
        if type(self.err_0) == type(None):
            self.err_0 = err.copy()

        # Exponential time constant for each update step
        exp = np.exp(-(1/self.y1)*dt)

        # Update e_fs
        self.err_f = exp*self.err_f + (1-exp)*err

        # Update h
        self.h = exp*self.h + (1-exp) * np.reshape(np.matmul(J, np.transpose(V_cam)), [4,2])

        # Now calculate g(t)
        if self.err_0 is not None:
            g = 1/self.y1 * (err - self.err_f)#  - np.exp(-(1/self.y1)*(self.it*dt)) * self.err_0) # Including crummy elapsed time estimate

        else:
            g = 1/self.y1 * (err - self.err_f)

        # Then solve estimate variable S
        S = np.array(self.h - g)

        # Estimate of target velocity
        Vp = np.matmul(np.linalg.pinv(J), S.flatten())
        self.Vp = Vp

        # Anti-peaking implementation (projection based estimator)

        # Compute phi
        phi = (1/self.y2) * (Vp - self.Vp_hat)

        # Projection condition check 
        f = (np.linalg.norm(self.Vp_hat)**2 - self.h1**2) / (self.eps0 * self.h1**2)
        grad_f = (2 * self.Vp_hat) / (self.eps0 * self.h1**2)

        # Apply correction estimate to keep correction bounded
        if f > 0 and np.matmul(np.transpose(phi), grad_f) > 0:
            frac = np.matmul(grad_f, np.transpose(grad_f)) / np.linalg.norm(self.Vp_hat)
            phi_f_term = phi * f
            phi = phi - frac * phi_f_term

        self.Vp_hat = self.Vp_hat + phi * dt
