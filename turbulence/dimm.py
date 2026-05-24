r"""
Differential Image Motion Monitor (DIMM) design-time toolset.

This module provides closed-form expressions and simple algebra for picking
a DIMM mask geometry :math:`(D, B)` and sanity-checking beacon SNR before
machining hardware. It deliberately omits the analysis-pipeline machinery
(bias corrections, covariance method, :math:`\tau_0` extraction) -- those
will live in a companion analysis module once the instrument is built.

The standard DIMM equation, valid for both plane and spherical waves
(with the appropriate :math:`r_0`),

.. math::
    \sigma^2_{l,t} = K_{l,t}(b) \, \left(\frac{\lambda}{D}\right)^2
                    \, \left(\frac{D}{r_0}\right)^{5/3}

with :math:`b = B/D` the dimensionless baseline ratio. The coefficients
:math:`K_l` and :math:`K_t` are the longitudinal and transverse response
functions from [Tokovinin2007]_.

Conventions
-----------
All quantities are SI:

==========================  ===============================
``D``, ``B``, ``lam``, ``focal_length``  metres
``sigma2`` (variances)      [rad\ :sup:`2`]
``exposure``, ``frame_period``           seconds
``irradiance``, ``power``                W m\ :sup:`-2`, W
all angles                  radians
==========================  ===============================

The ``motion`` argument selects which of the differential AoA components
is being computed:

- ``'longitudinal'`` -- along the baseline (:math:`K_l`, larger variance).
- ``'transverse'`` -- perpendicular to the baseline (:math:`K_t`).
- ``'combined'`` -- :math:`K_l + K_t` (the c-motion), preferred for
  :math:`r_0` retrieval because its wind-direction dependence is
  weaker than 2% per [Kornilov2011]_ §3.2.

References
----------
.. [Tokovinin2002] Tokovinin, A. (2002), "From Differential Image Motion
    to Seeing", PASP 114, 1156.
.. [Tokovinin2007] Tokovinin, A. and Kornilov, V. (2007), "Accurate seeing
    measurements with MASS and DIMM", MNRAS 381, 1179.
.. [Kornilov2011] Kornilov, V. and Safonov, B. (2011), "Differential image
    motion in the short-exposure regime", MNRAS 418, 1878.
"""

import numpy as np


# ===========================================================================
# 1. Kernel coefficients -- Tokovinin & Kornilov 2007 closed forms
# ===========================================================================
def K_longitudinal(b):
    r"""
    Differential AoA variance coefficient parallel to the baseline.

    .. math::
        K_l(b) = 0.340 \left(1 - 0.570 \, b^{-1/3}
                              - 0.040 \, b^{-7/3}\right)

    Parameters
    ----------
    b : float or array_like
        Baseline-to-aperture ratio :math:`B/D`.

    Returns
    -------
    K_l : float or ndarray
        Longitudinal response coefficient.

    Notes
    -----
    G-tilt form from [Tokovinin2007]_ Eq. 8, accurate to better than 0.5%
    for :math:`b > 1.2`.

    See Also
    --------
    K_transverse, K_combined
    """
    b = np.asarray(b, dtype=float)
    return 0.340 * (1.0 - 0.570 * b ** (-1.0 / 3.0) - 0.040 * b ** (-7.0 / 3.0))


def K_transverse(b):
    r"""
    Differential AoA variance coefficient perpendicular to the baseline.

    .. math::
        K_t(b) = 0.340 \left(1 - 0.855 \, b^{-1/3}
                              + 0.030 \, b^{-7/3}\right)

    Parameters
    ----------
    b : float or array_like
        Baseline-to-aperture ratio :math:`B/D`.

    Returns
    -------
    K_t : float or ndarray
        Transverse response coefficient.

    Notes
    -----
    :math:`K_l > K_t` for all finite :math:`b` -- wavefront variations
    along the baseline contribute more to differential tilt than across
    it. At :math:`b = 2.5`, :math:`K_l / K_t = 1.54`, an isotropy
    diagnostic.
    """
    b = np.asarray(b, dtype=float)
    return 0.340 * (1.0 - 0.855 * b ** (-1.0 / 3.0) + 0.030 * b ** (-7.0 / 3.0))


def K_combined(b):
    r"""
    Combined response coefficient :math:`K_c = K_l + K_t` for c-motion.

    Parameters
    ----------
    b : float or array_like
        Baseline-to-aperture ratio :math:`B/D`.

    Returns
    -------
    K_c : float or ndarray
        Combined response coefficient.

    Notes
    -----
    The variance of the c-motion :math:`\sigma_l^2 + \sigma_t^2` varies
    by less than 2% with wind direction per [Kornilov2011]_ §3.2, making
    it the recommended quantity for :math:`r_0` retrieval on links with
    unknown or variable wind direction.
    """
    return K_longitudinal(b) + K_transverse(b)


def _K_for_motion(b, motion):
    """Internal dispatcher: pick the right K coefficient for a motion label."""
    if motion == "longitudinal":
        return K_longitudinal(b)
    if motion == "transverse":
        return K_transverse(b)
    if motion == "combined":
        return K_combined(b)
    raise ValueError(
        f"motion must be 'longitudinal' / 'transverse' / 'combined', "
        f"got {motion!r}"
    )


# ===========================================================================
# 2. Forward model -- r0 -> predicted measured variance
# ===========================================================================
def predicted_AoA_variance(r0, lam, D, b, motion="longitudinal"):
    r"""
    Predict differential AoA variance from Fried parameter (forward model).

    .. math::
        \sigma^2 = K(b) \, \left(\lambda / D\right)^2
                   \, \left(D / r_0\right)^{5/3}

    Parameters
    ----------
    r0 : float
        Fried parameter [m]. Pass the plane-wave value to obtain the
        plane-wave variance, the spherical-wave value to obtain the
        spherical-wave variance.
    lam : float
        Optical wavelength [m].
    D : float
        Sub-aperture diameter [m].
    b : float
        Baseline ratio :math:`B/D`.
    motion : {'longitudinal', 'transverse', 'combined'}, optional
        Which differential AoA component to predict.

    Returns
    -------
    sigma2 : float
        Predicted differential AoA variance [rad\ :sup:`2`].

    Notes
    -----
    This equation is wave-agnostic in the sense that the plane- vs
    spherical-wave geometry is absorbed entirely into the definition of
    :math:`r_0` (see :mod:`turbulence`). The same :math:`K(b)` applies
    in both cases.
    """
    K = _K_for_motion(b, motion)
    return K * (lam / D) ** 2 * (D / r0) ** (5.0 / 3.0)


def predicted_centroid_displacement_rms(r0, lam, D, b, motion="longitudinal"):
    r"""
    RMS sky-frame centroid displacement from Fried parameter.

    Convenience wrapper that returns :math:`\sqrt{\sigma^2}` from
    :func:`predicted_AoA_variance`.

    Parameters
    ----------
    r0 : float
        Fried parameter [m].
    lam : float
        Optical wavelength [m].
    D : float
        Sub-aperture diameter [m].
    b : float
        Baseline ratio :math:`B/D`.
    motion : {'longitudinal', 'transverse', 'combined'}, optional
        Which differential AoA component to predict.

    Returns
    -------
    sigma : float
        RMS centroid displacement on sky [rad].
    """
    return np.sqrt(predicted_AoA_variance(r0, lam, D, b, motion=motion))


# ===========================================================================
# 3. Mask geometry -- does this fit? where do the spots land?
# ===========================================================================
def mask_fits_in_pupil(D, B, pupil_diameter, wall_thickness=2e-3):
    """
    Verify a candidate DIMM mask fits inside the telescope pupil.

    Two sub-apertures of diameter ``D`` separated by centre-to-centre
    baseline ``B`` must fit inside a circular pupil with at least
    ``wall_thickness`` clearance between the holes and from the pupil
    edge.

    Parameters
    ----------
    D : float
        Sub-aperture diameter [m].
    B : float
        Centre-to-centre baseline [m].
    pupil_diameter : float
        Telescope clear-aperture diameter [m].
    wall_thickness : float, optional
        Minimum allowed wall thickness (between holes and to pupil edge),
        defaults to 2 mm.

    Returns
    -------
    result : dict
        With keys:

        ``'fits'`` : bool
            True if both clearance constraints are satisfied.
        ``'edge_clearance'`` : float
            Distance from outer hole edge to pupil edge [m].
            Negative means the hole extends beyond the pupil.
        ``'inter_hole_gap'`` : float
            Distance between the two hole edges [m].
            Negative means the holes overlap.
        ``'wall_thickness'`` : float
            The minimum required clearance used.
    """
    outer_hole_edge_radius = B / 2.0 + D / 2.0
    edge_clearance = pupil_diameter / 2.0 - outer_hole_edge_radius
    inter_hole_gap = B - D
    fits = (edge_clearance >= wall_thickness) and (inter_hole_gap >= wall_thickness)
    return {
        "fits": fits,
        "edge_clearance": edge_clearance,
        "inter_hole_gap": inter_hole_gap,
        "wall_thickness": wall_thickness,
    }


def wedge_deflection_angle(wedge_apex_angle, refractive_index=1.510):
    r"""
    Thin-prism deflection angle.

    .. math::
        \theta_\mathrm{def} = (n - 1) \, \theta_\mathrm{apex}

    Parameters
    ----------
    wedge_apex_angle : float
        Physical apex angle of the wedge [rad].
    refractive_index : float, optional
        Wedge material refractive index at the operating wavelength.
        Defaults to 1.510 (N-BK7 at 850 nm). Use 1.453 for UV-grade
        fused silica, 1.760 for sapphire.

    Returns
    -------
    deflection : float
        Beam deflection angle in air [rad].

    Notes
    -----
    The thin-prism approximation is valid for apex angles up to a few
    degrees. For PS810-B (apex 2°) the small-angle assumption holds.
    """
    return (refractive_index - 1.0) * wedge_apex_angle


def spot_separation_on_detector(focal_length, wedge_deflection):
    """
    Linear separation of pupil images at the focal plane.

    When one sub-aperture has a wedge prism with the given deflection
    angle, the two pupil images are separated at the detector by

    .. math::
        s = f \, \theta_\mathrm{def}

    Parameters
    ----------
    focal_length : float
        Effective focal length to the detector [m].
    wedge_deflection : float
        Beam deflection angle from :func:`wedge_deflection_angle` [rad].

    Returns
    -------
    separation : float
        Spot separation at the focal plane [m].

    Notes
    -----
    Without a wedge or other beam-splitting element, the two pupil
    images coincide and the centroid difference is identically zero
    -- the wedge is what makes the DIMM measurable.

    Example: PS810-B at 850 nm gives ``wedge_deflection = 17.8 mrad``.
    With ``focal_length = 0.075`` m (relay-reduced), separation =
    1.34 mm; with ``focal_length = 0.600`` m (native), 10.7 mm.
    """
    return focal_length * wedge_deflection


# ===========================================================================
# 4. Beacon SNR -- photons per sub-aperture per frame
# ===========================================================================
def irradiance_at_receiver_from_isotropic_beacon(power_in_band, range_m,
                                                  full_divergence_angle=None):
    r"""
    Optical irradiance at the receiver pupil from a distant beacon.

    Parameters
    ----------
    power_in_band : float
        Beacon power emitted in the filter passband [W]. This is the
        relevant number for SNR; not the beacon's total optical output.
    range_m : float
        Source-to-receiver distance [m].
    full_divergence_angle : float, optional
        Full-angle beacon emission cone [rad]. ``None`` (default) treats
        the source as isotropic (worst case).

    Returns
    -------
    irradiance : float
        Optical irradiance at the receiver pupil [W m\ :sup:`-2`].

    Notes
    -----
    Two geometries supported:

    - Isotropic (``full_divergence_angle=None``):
      :math:`E = P / (4\pi L^2)`.
    - Cone (``full_divergence_angle = theta``):
      :math:`E = P / [\pi (L \theta / 2)^2]`,
      assuming uniform intensity across the cone.

    For an L-810 obstruction light (red, ~620 nm) used with an 850 nm
    bandpass filter, ``power_in_band`` is essentially zero -- specify
    a different filter or deploy an 850 nm LED beacon instead.
    """
    if full_divergence_angle is None:
        return power_in_band / (4.0 * np.pi * range_m ** 2)
    spot_radius = range_m * full_divergence_angle / 2.0
    return power_in_band / (np.pi * spot_radius ** 2)


def photons_per_subaperture_per_frame(irradiance_at_pupil, D, exposure,
                                       wavelength, transmission=0.5, qe=0.7):
    """
    Expected detected photon count per sub-aperture per frame.

    Parameters
    ----------
    irradiance_at_pupil : float
        In-band optical irradiance at the receiver pupil [W m\\ :sup:`-2`].
    D : float
        Sub-aperture diameter [m].
    exposure : float
        Per-frame exposure time [s].
    wavelength : float
        Optical wavelength [m], used for photon energy.
    transmission : float, optional
        End-to-end optical throughput (filter * telescope * wedge *
        detector window). Defaults to 0.5; 0.2 is more realistic.
    qe : float, optional
        Detector quantum efficiency at this wavelength. Defaults to 0.7.

    Returns
    -------
    photons : float
        Detected photon count per sub-aperture per frame.

    Notes
    -----
    Shot-noise-limited SNR per frame is :math:`\\sqrt{N_\\mathrm{ph}}`.
    Read noise becomes relevant only when ``photons < ~25`` for typical
    sCMOS detectors.
    """
    h = 6.62607015e-34
    c = 2.99792458e8
    photon_energy = h * c / wavelength
    pupil_area = np.pi * (D / 2.0) ** 2
    incident_photons = irradiance_at_pupil * pupil_area * exposure / photon_energy
    return incident_photons * transmission * qe


# ===========================================================================
# 5. Centroid noise -- photon-shot-noise floor on per-frame centroid
# ===========================================================================
def centroid_noise_rms_pixels(fwhm_pixels, snr_per_frame):
    r"""
    Photon-shot-noise centroid uncertainty.

    .. math::
        \sigma_\mathrm{cent} \approx \mathrm{FWHM} / \mathrm{SNR}

    Parameters
    ----------
    fwhm_pixels : float
        Spot FWHM on the detector [pixels].
    snr_per_frame : float
        Per-frame signal-to-noise ratio, typically
        :math:`\sqrt{N_\mathrm{photons}}` in the read-noise-negligible regime.

    Returns
    -------
    sigma : float
        Per-frame centroid uncertainty [pixels].

    Notes
    -----
    The simple SNR-based approximation from [Tokovinin2002]_ §3 is accurate
    to ~10% for properly thresholded centroids when SNR > 10.
    """
    return fwhm_pixels / snr_per_frame


def centroid_noise_variance_radians(fwhm_pixels, snr_per_frame,
                                     plate_scale_rad_per_pixel):
    r"""
    Per-frame centroid noise variance projected onto sky frame.

    Parameters
    ----------
    fwhm_pixels : float
        Spot FWHM on the detector [pixels].
    snr_per_frame : float
        Per-frame signal-to-noise ratio.
    plate_scale_rad_per_pixel : float
        Detector plate scale [rad pixel\ :sup:`-1`].

    Returns
    -------
    sigma2 : float
        Single-aperture centroid noise variance [rad\ :sup:`2`].

    Notes
    -----
    The *differential* AoA noise variance is :math:`2 \sigma^2_\mathrm{cent}`
    because the two sub-aperture centroids are independent. Subtract this
    before applying the DIMM inversion.
    """
    sigma_pixels = centroid_noise_rms_pixels(fwhm_pixels, snr_per_frame)
    return (sigma_pixels * plate_scale_rad_per_pixel) ** 2


# ===========================================================================
# 6. Dynamic range and validity
# ===========================================================================
def dimm_validity_regime(r0, D):
    """
    Classify the DIMM operating regime by :math:`D / r_0`.

    Parameters
    ----------
    r0 : float
        Fried parameter at the operating wavelength [m].
    D : float
        Sub-aperture diameter [m].

    Returns
    -------
    regime : str
        One of:

        - ``'weak_turbulence'`` (:math:`D/r_0 < 0.5`):
            atmospheric tilt small; signal-to-noise limited but valid.
        - ``'nominal'`` (:math:`0.5 \\le D/r_0 < 4`):
            canonical DIMM regime, well-behaved spots.
        - ``'strong_speckle'`` (:math:`4 \\le D/r_0 < 8`):
            spot becoming speckly, centroid bias starts to matter.
        - ``'saturated'`` (:math:`D/r_0 \\ge 8`):
            centroid unreliable; reduce ``D`` or switch to a
            scintillation channel.

    Notes
    -----
    The practical sweet spot is :math:`D \\sim 2 r_0` to :math:`4 r_0`
    per [Tokovinin2002]_ §3.
    """
    ratio = D / r0
    if ratio < 0.5:
        return "weak_turbulence"
    if ratio < 4.0:
        return "nominal"
    if ratio < 8.0:
        return "strong_speckle"
    return "saturated"


def rytov_variance_spherical_wave(Cn2, lam, L):
    r"""
    Spherical-wave Rytov log-amplitude variance, homogeneous :math:`C_n^2`.

    .. math::
        \sigma_R^2 = 0.5 \, C_n^2 \, k^{7/6} \, L^{11/6}

    Parameters
    ----------
    Cn2 : float
        Path-averaged refractive-index structure constant [m\ :sup:`-2/3`].
    lam : float
        Optical wavelength [m].
    L : float
        Path length [m].

    Returns
    -------
    sigma2_R : float
        Rytov variance (dimensionless).

    Notes
    -----
    The Rytov variance flags the validity of weak-fluctuation theory:
    :math:`\sigma_R^2 < 0.3` is the weak regime; :math:`\sigma_R^2 > 1`
    is strong-fluctuation regime where saturation effects can bias DIMM
    measurements even when :math:`D / r_0` looks acceptable.
    """
    k = 2.0 * np.pi / lam
    return 0.5 * Cn2 * k ** (7.0 / 6.0) * L ** (11.0 / 6.0)


def minimum_detectable_r0(D, lam, b, centroid_noise_variance_rad2, n_frames,
                            motion="longitudinal", snr_threshold=3.0):
    r"""
    Largest detectable :math:`r_0` for a given noise floor and frame count.

    Solves :math:`\sigma^2_\mathrm{signal} \ge \mathrm{threshold} \cdot
    \mathrm{noise\,floor}` for the largest :math:`r_0`.

    Parameters
    ----------
    D : float
        Sub-aperture diameter [m].
    lam : float
        Operating wavelength [m].
    b : float
        Baseline ratio :math:`B / D`.
    centroid_noise_variance_rad2 : float
        Single-aperture centroid noise variance [rad\ :sup:`2`].
    n_frames : int
        Number of independent samples per measurement block.
    motion : {'longitudinal', 'transverse', 'combined'}, optional
        Which AoA component sets the detection threshold.
    snr_threshold : float, optional
        Required ratio of signal to variance-estimator noise floor.
        Defaults to 3 (3-sigma detection).

    Returns
    -------
    r0_max : float
        Largest :math:`r_0` detectable above the noise floor [m].

    Notes
    -----
    The standard error of a sample variance from :math:`N` independent
    Gaussian samples is :math:`\sigma_\mathrm{noise}^2 \sqrt{2/N}`.
    Beyond ``r0_max`` the atmospheric tilt is buried in centroid noise.
    """
    K = _K_for_motion(b, motion)
    floor = snr_threshold * centroid_noise_variance_rad2 * np.sqrt(2.0 / n_frames)
    return D / (floor / (K * (lam / D) ** 2)) ** (3.0 / 5.0)


# ===========================================================================
# 7. One-shot design report
# ===========================================================================
def summarise_mask_design(D, B, pupil_diameter, lam, focal_length,
                            wedge_apex_angle, irradiance_at_pupil,
                            exposure, frame_count, plate_scale_rad_per_pixel,
                            fwhm_pixels, transmission=0.5, qe=0.7,
                            r0_range_cm=(2, 4, 8, 16, 32),
                            wedge_index=1.510, wall_thickness=2e-3):
    """
    Print a one-page design report for a candidate DIMM mask.

    Iterates over the design-time quantities (geometry, spot separation,
    photon budget, validity regime) and prints them as a single
    human-readable table. Intended for command-line mask iteration:
    tune ``D`` and ``B`` until the numbers look acceptable, then commit
    to machining.

    Parameters
    ----------
    D : float
        Sub-aperture diameter [m].
    B : float
        Centre-to-centre baseline [m].
    pupil_diameter : float
        Telescope clear-aperture diameter [m].
    lam : float
        Operating wavelength [m].
    focal_length : float
        Effective focal length to the detector [m].
    wedge_apex_angle : float
        Physical wedge apex angle [rad].
    irradiance_at_pupil : float
        In-band optical irradiance at the pupil [W m\\ :sup:`-2`].
    exposure : float
        Per-frame exposure time [s].
    frame_count : int
        Frames per measurement block (typical: 2000).
    plate_scale_rad_per_pixel : float
        Detector plate scale on sky [rad pixel\\ :sup:`-1`].
    fwhm_pixels : float
        Spot FWHM on detector [pixels].
    transmission : float, optional
        End-to-end optical throughput.
    qe : float, optional
        Detector quantum efficiency at ``lam``.
    r0_range_cm : tuple of float, optional
        Fried parameters [cm] to characterise across the validity table.
    wedge_index : float, optional
        Wedge material refractive index at ``lam``. Defaults to 1.510
        (N-BK7 at 850 nm).
    wall_thickness : float, optional
        Minimum allowed mask wall thickness [m].

    Returns
    -------
    None
        Output is printed to stdout.

    See Also
    --------
    mask_fits_in_pupil, predicted_AoA_variance, dimm_validity_regime,
    minimum_detectable_r0
    """
    b = B / D
    K_l = K_longitudinal(b)
    K_t = K_transverse(b)
    K_c = K_combined(b)

    geom = mask_fits_in_pupil(D, B, pupil_diameter, wall_thickness=wall_thickness)
    deflection = wedge_deflection_angle(wedge_apex_angle, wedge_index)
    spot_sep = spot_separation_on_detector(focal_length, deflection)

    n_phot = photons_per_subaperture_per_frame(
        irradiance_at_pupil, D, exposure, lam,
        transmission=transmission, qe=qe,
    )
    snr_per_frame = np.sqrt(max(n_phot, 1.0))  # avoid divide-by-zero at SNR=0
    sigma_noise_px = centroid_noise_rms_pixels(fwhm_pixels, snr_per_frame)
    sigma_noise_rad2 = centroid_noise_variance_radians(
        fwhm_pixels, snr_per_frame, plate_scale_rad_per_pixel,
    )

    print("=" * 64)
    print(f"DIMM mask design summary -- D={D*1e3:.1f} mm, B={B*1e3:.1f} mm")
    print("=" * 64)
    print(f"  baseline ratio b              = {b:.3f}")
    print(f"  K_l, K_t, K_combined          = {K_l:.4f}, {K_t:.4f}, {K_c:.4f}")
    print(f"  K_l/K_t (isotropy diagnostic) = {K_l/K_t:.3f} "
          f"(expect 1.54 at b=2.5)")
    print()
    print(f"  fits in pupil (Ø{pupil_diameter*1e3:.0f} mm, "
          f">={wall_thickness*1e3:.1f} mm walls)? {geom['fits']}")
    print(f"    edge clearance              = {geom['edge_clearance']*1e3:+7.2f} mm")
    print(f"    inter-hole gap              = {geom['inter_hole_gap']*1e3:+7.2f} mm")
    print()
    print(f"  wedge deflection (n={wedge_index})  = {deflection*1e3:.3f} mrad")
    print(f"  spot separation on detector   = {spot_sep*1e3:.3f} mm "
          f"(at F={focal_length*1e3:.0f} mm)")
    print()
    print(f"  photons/sub-aperture/frame    = {n_phot:.1f} (SNR ~ {snr_per_frame:.1f})")
    print(f"  centroid noise (per frame)    = {sigma_noise_px:.3f} px = "
          f"{np.sqrt(sigma_noise_rad2)*1e6:.2f} µrad")
    print()
    print(f"  validity regime across r0:")
    print(f"    {'r0 [cm]':>9}  {'D/r0':>6}  {'regime':>16}  "
          f"{'σ_l [µrad]':>12}  {'signal/noise':>14}")
    for r0_cm in r0_range_cm:
        r0 = r0_cm / 100.0
        regime = dimm_validity_regime(r0, D)
        sig2_l = predicted_AoA_variance(r0, lam, D, b, "longitudinal")
        sig_l = np.sqrt(sig2_l)
        # Differential noise floor is 2x single-aperture variance
        snr_signal = sig2_l / (2.0 * sigma_noise_rad2)
        print(f"    {r0_cm:>9.1f}  {D/r0:>6.2f}  {regime:>16s}  "
              f"{sig_l*1e6:>12.2f}  {snr_signal:>14.1f}")
    print()
    r0_max = minimum_detectable_r0(D, lam, b, sigma_noise_rad2, frame_count,
                                    motion="longitudinal", snr_threshold=3.0)
    print(f"  minimum detectable r0 (3σ, N={frame_count} frames) = "
          f"{r0_max*100:.2f} cm")
    print("=" * 64)


# ===========================================================================
# Command-line interface
# ===========================================================================
def _cli():
    """Command-line wrapper -- ``python -m turbulence.dimm --help``."""
    import argparse
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--D", type=float, required=True,
                   help="Sub-aperture diameter [m]")
    p.add_argument("--B", type=float, required=True,
                   help="Centre-to-centre baseline [m]")
    p.add_argument("--pupil", type=float, default=0.080,
                   help="Telescope clear-aperture diameter [m]")
    p.add_argument("--lam", type=float, default=850e-9,
                   help="Wavelength [m]")
    p.add_argument("--F", type=float, default=0.600,
                   help="Effective focal length to detector [m]")
    p.add_argument("--wedge-deg", type=float, default=2.0,
                   help="Wedge apex angle [deg]")
    p.add_argument("--irradiance", type=float, default=1e-9,
                   help="In-band irradiance at pupil [W m^-2]")
    p.add_argument("--exposure", type=float, default=2e-3,
                   help="Frame exposure [s]")
    p.add_argument("--n-frames", type=int, default=2000,
                   help="Frames per measurement block")
    p.add_argument("--plate-scale-urad", type=float, default=8.0,
                   help="Detector plate scale [µrad/pixel]")
    p.add_argument("--fwhm-px", type=float, default=2.5,
                   help="Spot FWHM on detector [pixels]")
    args = p.parse_args()

    summarise_mask_design(
        D=args.D, B=args.B, pupil_diameter=args.pupil, lam=args.lam,
        focal_length=args.F,
        wedge_apex_angle=np.deg2rad(args.wedge_deg),
        irradiance_at_pupil=args.irradiance,
        exposure=args.exposure,
        frame_count=args.n_frames,
        plate_scale_rad_per_pixel=args.plate_scale_urad * 1e-6,
        fwhm_pixels=args.fwhm_px,
    )


if __name__ == "__main__":
    _cli()
