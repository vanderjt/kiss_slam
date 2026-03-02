# Notebooks

## Zero-to-Hero Notebook

Path: `notebooks/ntbk-zero-to-hero-kiss-slam.ipynb`

This notebook is a practical EKF-SLAM walkthrough using the real `kiss_slam` classes:
`World2D`, `Simulator`, motion/measurement models, `EKFSLAM`, data association, and `LiveViewer`.

## Run with Conda + Jupyter

```bash
conda create -n kiss_slam python=3.10 -y
conda activate kiss_slam
pip install -e .
conda install -y jupyter matplotlib numpy scipy
jupyter notebook
```

Then open:

- `notebooks/ntbk-zero-to-hero-kiss-slam.ipynb`

## Notes

- Execute the notebook **top-to-bottom** in a fresh kernel.
- The notebook sets random seeds for reproducibility.
- Total runtime is designed to stay under ~2 minutes on common laptops.
