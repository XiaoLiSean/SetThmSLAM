# SetThmSLAM
Set theoretical localization and mapping: This project aims to robustly localize autonomous vehicles in public space installed with infrastructure sensors. With limited information regarding vehicle dynamics, a centralized control system needs to properly predict the vehicle motion and update the estimated vehicle states with acquired measurements. Previous works have addressed localization using onboard sensors using both set and probabilistic methods, yet the infrastructure-based localization is more readily embedded with centralized multi-agent controls. This paper presents a set-theoretical algorithm for centralized infrastructure-based vehicle localization. We evaluate the proposed method in both simulation and real-world experiments. We demonstrate our method outperforms the FastSLAM by being less sensitive to initialization and achieving a guaranteed localization where the target nominal states are bounded in the corresponding uncertainties sets.

## Demonstration in Simulation

### Parking space installed with monocular cameras
https://user-images.githubusercontent.com/58400416/132111036-cb027901-9113-4160-91f9-ee128a662901.mp4

### Parking space installed with stereo cameras
https://user-images.githubusercontent.com/58400416/132111045-b5ebb511-ef98-4d88-be7f-5a13cc8706f8.mp4
