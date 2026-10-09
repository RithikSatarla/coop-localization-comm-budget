# coop-localization-comm-budget

How much inter-robot communication does cooperative localization actually
need? Cooperative localization lets a robot improve its pose estimate by
observing a teammate and fusing the teammate's communicated estimate, but
every such fusion costs a message. On the UTIAS MR.CLAM multi-robot datasets
(five robots, real odometry, camera range/bearing to landmarks and to each
other, Vicon ground truth) this repository constrains how often robots may use
teammate messages, through random loss, a communication radius and a rate
limit, and maps localization accuracy against the number of messages used.

## Headline finding

With all 15 landmarks in view, teammate messages do not improve
localization: team position RMSE is 0.152 m with 53 messages per robot per
minute and 0.153 m with none, and every constrained setting lies within
0.149-0.153 m. When three of the five robots have no landmark access, teammate
messages cut their RMSE from 4.45 m to 0.72 m, and a budget of one teammate
update per robot per 10 s (2.5 messages per robot per minute, 4.6 % of the
available messages) recovers 99.7 % of that gain. At equal budgets, regularly
spaced updates beat random delivery, and a short communication radius is the
worst way to spend the budget.

![Accuracy versus messages used](figures/accuracy_vs_messages.png)

*Team position RMSE against teammate messages delivered per robot per minute,
pooling all three constraint families and averaged over MR.CLAM datasets 1-4.
Left: all robots observe landmarks. Right: robots 3-5 have no landmark access
and localize from odometry and teammate messages only. The star marks the
knee of the lower envelope.*

Full tables, all figures, findings and limitations: [docs/results.md](docs/results.md).

## Methods

* **Dead reckoning**: integrates the logged velocity commands from the
  ground-truth initial pose.
* **EKF with landmarks**: unicycle EKF fusing odometry with range/bearing to
  the 15 landmarks at their known positions.
* **Cooperative EKF**: the same filter run jointly for all five robots, which
  additionally fuses range/bearing to teammates using the teammate's current
  estimate as the observed point; the teammate's uncertainty is folded in with
  split covariance intersection so the decoupled filters stay consistent.
* **Communication constraints** on the cooperative EKF: message drop
  probability p in {0, 0.25, 0.5, 0.75, 0.9, 1}; comm radius on measured range
  in {1, 2, 4, 8 m, unlimited}; at most one teammate update per robot per
  {1, 5, 10, 30 s, never}. Every run logs the messages delivered.
* **Two regimes**: the standard full-map task, and a map-blind team in which
  only robots 1-2 see landmarks (added because, with 15 landmarks in view,
  cooperation has little left to improve).

Ground truth is used only for the initial pose and for scoring. Ground-truth
dropout windows (Vicon gaps longer than 0.5 s) are excluded from scoring.

## Results at a glance

Mean over MR.CLAM datasets 1-4; "messages" are teammate messages delivered.
Dead reckoning alone: 4.15 m.

| setting | messages per robot per minute | share of messages | team RMSE, full map [m] | team RMSE, map-blind team [m] |
|---|---|---|---|---|
| no messages (landmark EKF) | 0 | 0 % | 0.153 | 2.722 |
| one update per 30 s | 1.1 | 2.1 % | 0.152 | 0.735 |
| one update per 10 s (knee) | 2.5 | 4.6 % | 0.150 | 0.492 |
| one update per 5 s | 4.2 | 7.9 % | 0.151 | 0.445 |
| one update per 1 s | 13.5 | 25.5 % | 0.150 | 0.428 |
| random loss p = 0.9 | 5.4 | 10.1 % | 0.151 | 0.696 |
| random loss p = 0.5 | 26.6 | 49.9 % | 0.150 | 0.533 |
| comm radius 2 m | 17.6 | 32.9 % | 0.149 | 1.003 |
| comm radius 4 m | 44.9 | 84.0 % | 0.151 | 0.558 |
| all messages | 53.2 | 100 % | 0.152 | 0.485 |

## Reproduce

```
pip install -r requirements.txt
python scripts/download_mrclam.py
python run_experiments.py
```

The third command runs all 344 filter runs (4 datasets x 2 regimes x
43 configurations) in about 14 minutes on 4 worker processes and writes
`results/*.csv`, `figures/*.{pdf,png}` and `results/tables.md`.
`python run_experiments.py --replot` regenerates the figures and tables from
the saved CSVs; `python -m unittest discover -s tests` runs 13 sanity tests on
a synthetic team (no data needed).

## Repository layout

```
scripts/download_mrclam.py        download + extract MR.CLAM datasets 1-4 (FTP, stdlib only)
scripts/measurement_residuals.py  sensor-noise diagnostic against ground truth
src/loader.py                     parse .dat files, map barcodes, resample to a 0.02 s grid, mask GT dropouts
src/localize.py                   dead reckoning, landmark EKF, cooperative EKF (covariance intersection)
src/comm.py                       drop / radius / rate communication policies
src/analysis.py                   sweep aggregation, knee detection, markdown tables
src/plotting.py                   figures (PDF + PNG)
run_experiments.py                full protocol; --quick smoke test; --replot
tests/test_sanity.py              unit tests on a synthetic team
results/                          CSV outputs, tables.md, run_metadata.json
figures/                          all figures
docs/results.md                   write-up
```

## Data

`scripts/download_mrclam.py` fetches `MRCLAM1.zip` to `MRCLAM4.zip` from the
official FTP server `ftp://asrl3.utias.utoronto.ca/MRCLAM/` (the dataset page
lists them as "Dataset 1-4"; the files on the server are named `MRCLAM<N>.zip`,
and dataset 4 extracts to a folder called `MRSLAM_Dataset4`). The download
worked directly over FTP; no mirror was needed. Data are not committed
(`data/` is git-ignored).

Dataset page: https://asrl.utias.utoronto.ca/datasets/mrclam/index.html

## Citation

If you use the data, cite the dataset paper:

> K. Y. K. Leung, Y. Halpern, T. D. Barfoot and H. H. T. Liu. "The UTIAS
> Multi-Robot Cooperative Localization and Mapping Dataset." International
> Journal of Robotics Research, 30(8):969-974, July 2011.

Covariance intersection for teammate fusion follows S. J. Julier and
J. K. Uhlmann, "General decentralized data fusion with covariance intersection
(CI)," Handbook of Multisensor Data Fusion, ch. 12, CRC Press, 2001, and
L. C. Carrillo-Arce, E. D. Nerurkar, J. L. Gordillo and S. I. Roumeliotis,
"Decentralized multi-robot cooperative localization using covariance
intersection," IROS 2013.

## License

MIT, see [LICENSE](LICENSE).
