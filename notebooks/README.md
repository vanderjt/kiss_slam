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

This sequence will be expanded as more notebooks are added:

1. `ntbk-zero-to-hero-kiss-slam.ipynb` (current end-to-end overview)
2. Upcoming theory notebooks in `ntbk-XX-...` order

