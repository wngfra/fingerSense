import jax
import jax.numpy as jnp
import jax.numpy.linalg as LA
from jax import random
from jax import jit

from utility import error_ellipsoid, KL_divergence_normal


@jit
def KL_div_normal(y, p, q, y0, n):
    mu_p, mu_q = p[0], q[0]
    sigma_p, sigma_q = p[1], q[1]

    mu_p_new = mu_p + (y - y0)/n
    sigma_p_new = (n-1)/n*sigma_p + jnp.outer(y - mu_p_new, y - mu_p)/n
    
    k = y.shape[0]
    return 0.5 * (jnp.log(LA.det(sigma_q)/LA.det(sigma_p_new)) - k + jnp.dot(jnp.dot((mu_p_new - mu_q).transpose(), LA.inv(sigma_q)), (mu_p_new - mu_q)) + jnp.trace(jnp.dot(LA.inv(sigma_q), sigma_p_new)))


if __name__ == '__main__':
    key = random.PRNGKey(0)
    p = [random.normal(key, [3, 1]) + 100, random.normal(key, [3, 3]) + 10]
    
    y0 = random.normal(key, [3, 1]) + 200
    y = random.normal(key, [3, 1]) + 50
    n = 32
    jacobian = jax.jacfwd(KL_div_normal, 0)

    def func(i):
        key = random.PRNGKey(i)
        q = [random.normal(key, [3, 1]) + 100, random.normal(key, [3, 3]) + 10]
        gradient = jacobian(y, p, q, y0, n)
        print("id: {}, {}".format(i, gradient.reshape(3)))

    for i in range(50):
        func(i)