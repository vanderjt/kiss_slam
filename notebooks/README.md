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
  - tiny reusable helpers (seed, angle wrap, shape checks, matrix printing,
    covariance ellipse plotting).

### How to use the template

1. Copy section structure from `_TEMPLATE_THEORY_NOTEBOOK.md`.
2. Create notebook with naming style: `ntbk-XX-topic-name.ipynb`.
3. Start with a reproducible setup cell (fixed seed).
4. Keep examples deterministic, lightweight, and runnable top-to-bottom.
5. Include exercises and a short recap.

---

## Recommended Learning Order

1. `ntbk-01-gaussians-and-uncertainty.ipynb`
2. `ntbk-02-multivariate-gaussians-covariance.ipynb`
3. `ntbk-03-covariance-ellipses-and-geometry.ipynb`
4. `ntbk-04-linearization-and-taylor-approx.ipynb`
5. `ntbk-05-jacobians-for-robotics.ipynb`
6. `ntbk-06-ekf-core-ideas-predict-update.ipynb`
7. `ntbk-07-ekf-2d-tracking-example.ipynb`
8. `ntbk-08-ekf-consistency-nis-nees.ipynb`
9. `ntbk-09-differential-drive-kinematics.ipynb`
10. `ntbk-10-motion-model-jacobians-and-process-noise.ipynb`
11. `ntbk-11-range-bearing-measurements.ipynb`
12. `ntbk-12-measurement-jacobians-and-innovation.ipynb`
13. `ntbk-13-data-association-and-gating.ipynb`
14. `ntbk-zero-to-hero-kiss-slam.ipynb` (end-to-end project view)
