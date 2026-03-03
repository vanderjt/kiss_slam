# Notebooks

This folder contains teaching notebooks for EKF-SLAM theory and practice.

## Run Notebooks (Conda + Jupyter)

```bash
conda create -n kiss_slam python=3.10 -y
conda activate kiss_slam
pip install -e .
conda install -y jupyter numpy matplotlib scipy
jupyter notebook
```

Then open any notebook under `notebooks/`.

---

## Theory Notebook Authoring Workflow

Use these files together when creating new theory notebooks:

- `notebooks/_TEMPLATE_THEORY_NOTEBOOK.md`
  - canonical structure and section order,
  - writing checklist for consistency.
- `notebooks/_notebook_utils.py`
  - reusable helpers (`set_seed`, angle wrapping, shape checks, matrix printing,
    covariance ellipse plotting).

### How to use the template

1. Copy section structure from `_TEMPLATE_THEORY_NOTEBOOK.md`.
2. Create notebook with naming style: `ntbk-XX-topic-name.ipynb`.
3. Start with a reproducible setup cell (fixed seed + matplotlib defaults).
4. Keep examples deterministic, lightweight, and runnable top-to-bottom.
5. Include exercises and a clearly marked solution section.

---

## Ordered Notebook Series

1. `ntbk-01-gaussians-and-uncertainty.ipynb` — scalar Gaussian intuition, sampling, and uncertainty interpretation.
2. `ntbk-02-multivariate-gaussians-covariance.ipynb` — covariance structure, correlation, and matrix-based uncertainty.
3. `ntbk-03-covariance-ellipses-and-geometry.ipynb` — geometric view of covariance via eigenvectors/eigenvalues.
4. `ntbk-04-linearization-and-taylor-approx.ipynb` — nonlinear functions, first-order approximations, and local error.
5. `ntbk-05-jacobians-for-robotics.ipynb` — Jacobian intuition and practical derivative checks.
6. `ntbk-06-ekf-core-ideas-predict-update.ipynb` — EKF predict/update cycle and covariance propagation.
7. `ntbk-07-ekf-2d-tracking-example.ipynb` — compact EKF tracking walkthrough in 2D.
8. `ntbk-08-ekf-consistency-nis-nees.ipynb` — NIS/NEES consistency diagnostics and interpretation.
9. `ntbk-09-differential-drive-kinematics.ipynb` — differential-drive motion fundamentals.
10. `ntbk-10-motion-model-jacobians-and-process-noise.ipynb` — motion model Jacobians and process noise effects.
11. `ntbk-11-range-bearing-measurements.ipynb` — range/bearing sensing model and observability intuition.
12. `ntbk-12-measurement-jacobians-and-innovation.ipynb` — innovation residuals, measurement Jacobians, and update behavior.
13. `ntbk-13-data-association-and-gating.ipynb` — correspondence logic and Mahalanobis gating.
14. `ntbk-14-ekf-slam-state-augmentation.ipynb` — landmark state augmentation and covariance block structure.
15. `ntbk-15-capstone-full-ekf-slam-pipeline.ipynb` — integrated pipeline lab with tuning/debug prompts.
16. `ntbk-zero-to-hero-kiss-slam.ipynb` — practical end-to-end orientation that connects all components.
