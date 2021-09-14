# SetThmSLAM
Set theoretical localization and mapping: This project aims to robustly localize autonomous vehicles in public space installed with infrastructure sensors. With limited information regarding vehicle dynamics, a centralized control system needs to properly predict the vehicle motion and update the estimated vehicle states with acquired measurements. Previous works have addressed localization using onboard sensors using both set and probabilistic methods, yet the infrastructure-based localization is more readily embedded with centralized multi-agent controls. This paper presents a set-theoretical algorithm for centralized infrastructure-based vehicle localization. We evaluate the proposed method in both simulation and real-world experiments. We demonstrate our method outperforms the FastSLAM by being less sensitive to initialization and achieving a guaranteed localization where the target nominal states are bounded in the corresponding uncertainties sets.

## Demonstration in Simulation

1. Parking space installed with monocular cameras

https://user-images.githubusercontent.com/58400416/132111298-997d92d8-4976-430f-84e6-de40c355b503.mp4



2. Parking space installed with stereo cameras

https://user-images.githubusercontent.com/58400416/132111295-971f1e66-7ea4-41bc-b835-a6135dfe4d66.mp4



## Real-world lidar localization

1. Demonstration Video

https://user-images.githubusercontent.com/58400416/133294083-76bd6d9f-2807-4ab0-ba4e-ffa9abc69788.mp4






