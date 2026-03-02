# Theory Notebook Template (KISS)

Use this template for every theory notebook in `notebooks/` so structure and learning flow stay consistent.

---

## 0) Notebook Header

- **Title**
- **Notebook ID** (example: `ntbk-03-jacobians-linearization.ipynb`)
- **Estimated runtime** (target: < 2 minutes)
- **Dependencies** (`numpy`, `matplotlib`, optional local `kiss_slam` imports)

Example:

```text
# Jacobians and Linearization for EKF-SLAM
Notebook: ntbk-03-jacobians-linearization.ipynb
Runtime: ~90 seconds
Dependencies: numpy, matplotlib
```

---

## 1) Introduction

Briefly explain:

- what concept this notebook covers,
- why it matters for EKF-SLAM,
- where it appears in the project code.

Keep this section short and motivational.

---

## 2) Learning Objectives

List 3–6 concrete outcomes using action verbs.

Example:

- derive a Jacobian for a nonlinear function,
- implement numerical Jacobian checks,
- explain how linearization error affects EKF updates.

---

## 3) Prerequisites

State what the reader should already know.

Example:

- basic linear algebra (vectors/matrices),
- Gaussian mean/covariance basics,
- previous notebooks: `ntbk-01`, `ntbk-02`.

---

## 4) Setup Cell (Reproducible)

First executable cell should:

- import only needed packages,
- set a fixed random seed,
- configure plotting defaults if helpful.

Template:

```python
import numpy as np
import matplotlib.pyplot as plt

SEED = 7
np.random.seed(SEED)
print(f"Seed set to {SEED}")
```

---

## 5) Core Theory

Main teaching section. Break into small subsections.

Recommended pattern per subsection:

1. **Concept** (short explanation + equation),
2. **Code demo** (minimal runnable snippet),
3. **Interpretation** (what result means physically/statistically).

Guidelines:

- Keep math readable (Markdown equations + short commentary).
- Prefer multiple small cells over one long cell.
- Explicitly connect symbols to code variable names.

---

## 6) Worked Examples

Include at least 1–2 end-to-end examples using realistic numeric values.

Each worked example should include:

- problem statement,
- step-by-step computation,
- final takeaway.

Keep examples deterministic and lightweight.

---

## 7) Interactive Experiments

Add short “change this parameter and observe” blocks.

Suggested format:

- provide baseline parameters,
- list 2–3 knobs to vary,
- prompt learner to predict behavior before running.

Example prompts:

- “Increase process noise. What happens to covariance growth?”
- “Decrease measurement noise. How does Kalman gain change?”

---

## 8) Exercises

Include 3–5 exercises from easy to medium.

For each exercise:

- give a clear task,
- provide a starter code cell,
- optionally include a hidden/optional solution cell.

Solution style:

- short,
- commented,
- focused on reasoning (not clever tricks).

---

## 9) Recap

End with:

- 3–6 bullet summary of key ideas,
- common pitfalls,
- pointer to next notebook.

---

## 10) Quality Checklist (Before Committing)

- [ ] Notebook runs top-to-bottom in fresh kernel.
- [ ] Seed is fixed and outputs are reproducible.
- [ ] Runtime stays under 2 minutes.
- [ ] Explanations are concise and beginner-friendly.
- [ ] Equations match implementation.
- [ ] At least one exercise is included.
- [ ] Recap and next-step pointers are present.

