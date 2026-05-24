r"""
Kolmogorov turbulence physics primitives.

This module implements the standard Kolmogorov turbulence model and its
derived quantities -- the refractive-index power spectrum, the phase
structure function, Fried's coherence parameter :math:`r_0`, and
path-weighted integrals over inhomogeneous :math:`C_n^2(z)`.

Both plane-wave and spherical-wave forms are provided. For finite-distance
beacons on a horizontal path (e.g. a tower beacon over a free-space link),
use the ``*_spherical_wave`` variants -- the plane-wave forms underestimate
:math:`r_0` by a factor :math:`(8/3)^{3/5} \approx 1.78` in that regime.

Conventions
-----------
All quantities are SI:

==================  =============================================
``kappa``           angular spatial wavenumber [rad m\ :sup:`-1`]
``r0``, ``lam``, ``L``    metres
``Cn2``             [m\ :sup:`-2/3`]
``z``               metres along propagation path,
                    ``z=0`` at receiver and ``z=L`` at source
==================  =============================================

Sensor-specific routines (DIMM, scintillometer, link budget) live in
companion modules.

References
----------
.. [Kolmogorov1941] Kolmogorov, A.N. (1941), "Dissipation of Energy in
    Locally Isotropic Turbulence", Dokl. Akad. Nauk SSSR 32, 16.
.. [Fried1965] Fried, D.L. (1965), "Statistics of a Geometric
    Representation of Wavefront Distortion", JOSA 55, 1427.
.. [Cheon2007] Cheon, Y. and Muschinski, A. (2007), "Closed-form
    approximations for the angle-of-arrival variance of plane and
    spherical waves through homogeneous and isotropic turbulence",
    JOSA A 24, 415.
.. [Andrews2005] Andrews, L.C. and Phillips, R.L. (2005), "Laser Beam
    Propagation through Random Media", 2nd ed., SPIE Press.
"""

import numpy as np
from scipy.integrate import simpson


# ---------------------------------------------------------------------------
# Spectrum and structure function
# ---------------------------------------------------------------------------
def kolmogorov_index_spectrum_3D(kappa, Cn2):
    r"""
    Kolmogorov three-dimensional refractive-index power spectrum.

    .. math::
        \Phi_n(\kappa) = 0.033 \, C_n^2 \, \kappa^{-11/3}

    valid in the inertial subrange
    :math:`2\pi / L_0 \ll \kappa \ll 2\pi / l_0`.

    Parameters
    ----------
    kappa : array_like
        Angular spatial wavenumber [rad m\ :sup:`-1`]. Non-positive
        values return zero to suppress the origin singularity.
    Cn2 : float or array_like
        Refractive-index structure constant [m\ :sup:`-2/3`].

    Returns
    -------
    phi_n : ndarray
        Spectral density at each ``kappa`` [m\ :sup:`11/3`].

    Notes
    -----
    The numerical constant 0.033 follows the Tatarski normalisation used in
    [Andrews2005]_. Some texts use 0.0229 with cyclic spatial frequency
    :math:`f = \kappa / 2\pi`; the two conventions are equivalent under
    the Jacobian transformation.
    """
    kappa = np.asarray(kappa, dtype=float)
    safe = np.where(kappa > 0, kappa, np.inf)
    return 0.033 * Cn2 * safe ** (-11.0 / 3.0)


def kolmogorov_phase_structure_function(r, r0):
    r"""
    Phase structure function for a plane wave in Kolmogorov turbulence.

    .. math::
        D_\phi(r) = 6.88 \, (r / r_0)^{5/3}

    The Fried convention sets :math:`D_\phi(r_0) = 6.88` rad\ :sup:`2`,
    chosen so that an aperture of size :math:`r_0` admits roughly 1 rad
    of phase variance across it.

    Parameters
    ----------
    r : array_like
        Pupil-plane separation [m].
    r0 : float
        Fried coherence length [m].

    Returns
    -------
    D_phi : ndarray
        Phase structure function value at each ``r`` [rad\ :sup:`2`].

    Notes
    -----
    Use this both for predicting AoA variance and for validating the
    :math:`r_0` calibration of FFT-generated phase screens (sample
    point pairs from the screen, compute :math:`D_\phi`, compare to
    this curve).
    """
    return 6.88 * (np.asarray(r, dtype=float) / r0) ** (5.0 / 3.0)


# ---------------------------------------------------------------------------
# Path integrals -- handle inhomogeneous Cn2(z) over the link
# ---------------------------------------------------------------------------
def propagation_geometry_kernel(z, L, kind="dimm_spherical_wave"):
    r"""
    Geometric weighting :math:`w(z)` along a propagation path of length ``L``.

    The kernel arises purely from wave geometry (plane vs spherical) and
    is independent of the sensor consuming the integral. The same three
    kernels apply to AoA-variance integrals, scintillation Rytov integrals,
    and isoplanatic-angle integrals.

    Parameters
    ----------
    z : array_like
        Position along the propagation path [m], ``0 <= z <= L``.
    L : float
        Total path length [m].
    kind : {'plane_wave', 'dimm_spherical_wave', 'scint_spherical_wave'}, optional
        Which geometric weighting to apply.

        * ``'plane_wave'`` -- :math:`w(z) = 1`.  Source at infinity, or any
          quantity that samples the path uniformly.
        * ``'dimm_spherical_wave'`` -- :math:`w(z) = (z/L)^{5/3}`.
          Spherical-wave AoA kernel from [Cheon2007]_; weights the source
          (``z = L``) end of the path.
        * ``'scint_spherical_wave'`` -- :math:`w(z) = [(z/L)(1-z/L)]^{5/6}`.
          Spherical-wave Rytov scintillation kernel; weights the path
          midpoint.

    Returns
    -------
    w : ndarray
        Dimensionless weighting at each ``z``.

    Notes
    -----
    For a 20 km link with the source at ``z = L`` (e.g. a tower beacon)
    and the receiver at ``z = 0``, the ``dimm_spherical_wave`` kernel
    concentrates sensitivity at the source end while
    ``scint_spherical_wave`` peaks at the midpoint.

    Raises
    ------
    ValueError
        If ``kind`` is not one of the three recognised options.
    """
    u = np.asarray(z, dtype=float) / L
    if kind == "plane_wave":
        return np.ones_like(u)
    if kind == "dimm_spherical_wave":
        return u ** (5.0 / 3.0)
    if kind == "scint_spherical_wave":
        return (u * (1.0 - u)) ** (5.0 / 6.0)
    raise ValueError(f"Unknown propagation geometry kernel: {kind!r}")


def integrate_Cn2_along_path(Cn2_profile, z, kind="dimm_spherical_wave"):
    r"""
    Weighted path integral of :math:`C_n^2(z)`.

    .. math::
        I = \int_0^L C_n^2(z) \, w(z) \, dz

    where the geometry weighting :math:`w(z)` is selected by ``kind``.

    Parameters
    ----------
    Cn2_profile : array_like
        Refractive-index structure constant [m\ :sup:`-2/3`] sampled at
        the points ``z``.
    z : array_like
        Path positions [m], monotonically increasing from 0 to ``L``.
    kind : str, optional
        Geometry kernel; see :func:`propagation_geometry_kernel`.

    Returns
    -------
    I : float
        Weighted path integral [m\ :sup:`1/3`].

    Notes
    -----
    Sanity values for a homogeneous profile along path ``L``:

    =========================  =======================
    kind                       integral
    =========================  =======================
    ``'plane_wave'``           :math:`C_n^2 \, L`
    ``'dimm_spherical_wave'``  :math:`(3/8) \, C_n^2 \, L`
    ``'scint_spherical_wave'`` :math:`0.378 \, C_n^2 \, L`
    =========================  =======================

    The 0.378 follows from :math:`B(11/6, 11/6)/2` where :math:`B` is the
    Euler beta function.
    """
    z = np.asarray(z, dtype=float)
    Cn2 = np.asarray(Cn2_profile, dtype=float)
    if z.shape != Cn2.shape:
        raise ValueError("z and Cn2_profile must have the same shape")
    L = z[-1] - z[0]
    w = propagation_geometry_kernel(z - z[0], L, kind=kind)
    return simpson(Cn2 * w, x=z)


# ---------------------------------------------------------------------------
# Fried parameter r0 <-> Cn2 -- plane and spherical wave, HOMOGENEOUS Cn2
# ---------------------------------------------------------------------------
def fried_parameter_plane_wave(Cn2, lam, L):
    r"""
    Plane-wave Fried coherence parameter for homogeneous :math:`C_n^2`.

    .. math::
        r_0 = \left[ 0.423 \, k^2 \, C_n^2 \, L \right]^{-3/5},
        \qquad k = 2\pi / \lambda

    Parameters
    ----------
    Cn2 : float
        Path-averaged refractive-index structure constant [m\ :sup:`-2/3`].
    lam : float
        Optical wavelength [m].
    L : float
        Propagation path length [m].

    Returns
    -------
    r0 : float
        Plane-wave Fried parameter [m].

    Notes
    -----
    The constant 0.423 derives from
    :math:`8\pi^2 \cdot 0.033 \cdot \Gamma`-function factors that arise
    from integrating the Kolmogorov spectrum against a plane-wave path.

    For inhomogeneous :math:`C_n^2(z)`, use
    :func:`fried_parameter_from_profile_spherical_wave` instead.

    See Also
    --------
    fried_parameter_spherical_wave : finite-distance beacon case.
    """
    k = 2.0 * np.pi / lam
    return (0.423 * k ** 2 * Cn2 * L) ** (-3.0 / 5.0)


def Cn2_from_fried_plane_wave(r0, lam, L):
    """
    Recover plane-wave :math:`C_n^2` from a measured plane-wave :math:`r_0`.

    Inverse of :func:`fried_parameter_plane_wave`.

    Parameters
    ----------
    r0 : float
        Plane-wave Fried parameter [m].
    lam : float
        Optical wavelength [m].
    L : float
        Propagation path length [m].

    Returns
    -------
    Cn2 : float
        Path-averaged refractive-index structure constant [m\\ :sup:`-2/3`].
    """
    k = 2.0 * np.pi / lam
    return r0 ** (-5.0 / 3.0) / (0.423 * k ** 2 * L)


def fried_parameter_spherical_wave(Cn2, lam, L):
    r"""
    Spherical-wave Fried parameter for a finite-distance beacon.

    Applies the :math:`(8/3)^{3/5} \approx 1.78` factor from [Cheon2007]_:
    spherical-wave AoA variance is :math:`3/8` of the plane-wave value
    for the same :math:`C_n^2 \, L`, so the inferred :math:`r_0` grows
    by :math:`(8/3)^{3/5}`.

    .. math::
        r_{0,\mathrm{sph}} = (8/3)^{3/5} \, r_{0,\mathrm{pw}}

    Parameters
    ----------
    Cn2 : float
        Path-averaged refractive-index structure constant [m\ :sup:`-2/3`].
    lam : float
        Optical wavelength [m].
    L : float
        Source-to-receiver distance [m].

    Returns
    -------
    r0 : float
        Spherical-wave Fried parameter [m].

    Notes
    -----
    Physical picture: a point source at distance :math:`L` casts a
    converging cone onto the receiver pupil. At intermediate path
    position :math:`z`, the cone has transverse size :math:`D \cdot z/L`.
    Turbulence near the source samples a large cone (many independent
    refractive eddies); turbulence at the pupil samples a vanishing cone
    (essentially a single point). Integrating the resulting weighting
    against Kolmogorov statistics produces the :math:`3/8` factor.
    """
    return (8.0 / 3.0) ** (3.0 / 5.0) * fried_parameter_plane_wave(Cn2, lam, L)


def Cn2_from_fried_spherical_wave(r0, lam, L):
    r"""
    Recover the true path-averaged :math:`C_n^2` from a spherical-wave
    :math:`r_0`.

    Inverse of :func:`fried_parameter_spherical_wave`.

    Parameters
    ----------
    r0 : float
        Spherical-wave Fried parameter [m] -- the physically correct one
        for a finite-distance beacon.
    lam : float
        Optical wavelength [m].
    L : float
        Source-to-receiver distance [m].

    Returns
    -------
    Cn2 : float
        Path-averaged refractive-index structure constant [m\ :sup:`-2/3`].
    """
    k = 2.0 * np.pi / lam
    return (8.0 / 3.0) * r0 ** (-5.0 / 3.0) / (0.423 * k ** 2 * L)


# ---------------------------------------------------------------------------
# Convenience: spherical-wave r0 directly from an inhomogeneous Cn2(z) profile
# ---------------------------------------------------------------------------
def fried_parameter_from_profile_spherical_wave(Cn2_profile, z, lam):
    r"""
    Spherical-wave :math:`r_0` from an inhomogeneous :math:`C_n^2(z)` profile.

    .. math::
        r_{0,\mathrm{sph}} = \left[ 0.423 \, k^2 \,
            \int_0^L C_n^2(z) \, (z/L)^{5/3} \, dz \right]^{-3/5}

    Use this when fed a meteorologically predicted profile (e.g. from
    :math:`\Pi`-ML or NAVSLaM) along a heterogeneous link.

    Parameters
    ----------
    Cn2_profile : array_like
        :math:`C_n^2` values [m\ :sup:`-2/3`] at the sample points ``z``.
    z : array_like
        Path positions [m], monotonically increasing from 0 to ``L``.
    lam : float
        Optical wavelength [m].

    Returns
    -------
    r0 : float
        Spherical-wave Fried parameter [m].

    Notes
    -----
    The :math:`(z/L)^{5/3}` weighting in the path integral *is* the
    spherical-wave geometry -- the integral evaluates to
    :math:`(3/8) \, C_n^2 \, L` for a homogeneous profile, which when
    raised to the :math:`-3/5` power produces the
    :math:`(8/3)^{3/5} \approx 1.78` boost relative to the plane-wave
    result. No additional geometric prefactor is required; this function
    reduces to :func:`fried_parameter_spherical_wave` in the homogeneous
    limit.
    """
    k = 2.0 * np.pi / lam
    I = integrate_Cn2_along_path(Cn2_profile, z, kind="dimm_spherical_wave")
    return (0.423 * k ** 2 * I) ** (-3.0 / 5.0)


# ---------------------------------------------------------------------------
# Command-line interface
# ---------------------------------------------------------------------------
def _cli():
    """Command-line wrapper -- ``python -m turbulence.turbulence --help``."""
    import argparse
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--r0", type=float, help="Fried parameter [m]")
    p.add_argument("--Cn2", type=float, help="Index structure constant [m^(-2/3)]")
    p.add_argument("--lam", type=float, default=850e-9,
                   help="Wavelength [m], default 850 nm")
    p.add_argument("--L", type=float, required=True, help="Path length [m]")
    p.add_argument("--wave", choices=("plane", "spherical"), default="spherical",
                   help="Plane- vs spherical-wave inversion")
    args = p.parse_args()

    if (args.Cn2 is None) == (args.r0 is None):
        p.error("Provide exactly one of --r0 or --Cn2.")

    if args.Cn2 is not None:
        fn = (fried_parameter_spherical_wave if args.wave == "spherical"
              else fried_parameter_plane_wave)
        r0 = fn(args.Cn2, args.lam, args.L)
        seeing_arcsec = 0.98 * (args.lam / r0) * (180 / np.pi) * 3600
        print(f"r0 ({args.wave:9s}) = {r0*100:7.3f} cm")
        print(f"seeing (FWHM)        = {seeing_arcsec:7.3f} arcsec "
              f"at lam={args.lam*1e9:.0f} nm")
    else:
        fn = (Cn2_from_fried_spherical_wave if args.wave == "spherical"
              else Cn2_from_fried_plane_wave)
        Cn2 = fn(args.r0, args.lam, args.L)
        print(f"Cn2 ({args.wave:9s}) = {Cn2:9.3e} m^(-2/3) at L={args.L/1e3:.1f} km")


if __name__ == "__main__":
    _cli()
