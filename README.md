# IndoorGraphLocalization
IndoorGraphLocalization is a multi-user indoor localization system utilizing graph-based optimization for sensor fusion.
The repository contains the code, the dataset and the scripts making it possible to reproduce results of our research.

If you found code useful in your academic work, please cite:

    @article{SensorsMN,
      title={A Multi-user Personal Indoor Localization System Employing Graph-based Optimization},
      author={Nowicki, M. R. and Skrzypczynski, P.},
      journal={MDPI Sensors},
      volume={},
      number={},
      pages={},
      doi = {},
      year={2019}
     }

     
# Installation

The installation can be performed with the provided ``build.sh`` script:
```
cd IndoorGraphLocalization
chmod +x build.sh
./build.sh
```

# Reproducing results

The results presented in the article can be reproduced with provided script:
```
cd IndoorGraphLocalization
python scripts/run.py
```
After running the script, the directory ``results`` should contain four directories:
* iUC_false_mKP_1.0_sLE_false_run_0
* iUC_false_mKP_1.0_sLE_true_run_0
* iUC_true_mKP_1.0_sLE_false_run_0
* iUC_true_mKP_1.0_sLE_true_run_0

The iUC stands for connections between users, while sLE stands for step length estimation. 
Each directory should contain results and figures for the evaluated approach. 
After running the script, the console should present the summary of the obtained results:

```
Config                                  RMSE    AvgErr  Sigma   MaxErr  
iUC_false_mKP_1.0_sLE_false_run_0       6.24    2.42    5.76    85.67   
iUC_false_mKP_1.0_sLE_true_run_0        5.19    2.21    4.69    57.25   
iUC_true_mKP_1.0_sLE_false_run_0        4.77    2.16    4.25    42.41   
iUC_true_mKP_1.0_sLE_true_run_0         4.65    2.11    4.15    42.64   
--------------
iUC_false_mKP_1.0_sLE_false_run         6.24    2.42    5.76    85.67   
iUC_false_mKP_1.0_sLE_true_run          5.19    2.21    4.69    57.25   
iUC_true_mKP_1.0_sLE_false_run          4.77    2.16    4.25    42.41   
iUC_true_mKP_1.0_sLE_true_run           4.65    2.11    4.15    42.64  
```

# License

The code is available under the GPLv3 license. 
For the convenience, the directory EXTERNAL contains the code of the g2o library (https://github.com/RainerKuemmerle/g2o)
 used for the optimization that is available mostly under the BSD license. 
Some external dependencies of the g2o library are available under GPLv3, so please check EXTERNAL/g2o/README.md for details.
