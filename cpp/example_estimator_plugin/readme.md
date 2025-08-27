# Example estimator plugin:

## Estimation file structure
![estimator_dependency_tree (1)](https://hackmd.io/_uploads/B12NLvntgl.jpg)
The MRS system provides different level of abstraction for developing estimator plugin which are either individual state estimators of comple estimators of UAV state.

Generic estimators for individual states like altitude, lateral and heading are already implemented. 

StateGeneric combines this partial estimators.

StateGeneric can be made specific to perticular sensor setup with parameter modification from yaml files.