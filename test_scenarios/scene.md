## Environment Modeling Methodology 
### Scene Construction
| Component        | Source                 | Format | Details                  |
|------------------|------------------------|--------|--------------------------|
| **Tree Scenario**  | Real-world | `scene4_tree.ply` | High-fidelity 3D reconstruction |
| **Other Scenarios**  | SolidWorks CAD design | `scene1, 2, 3*.ply` | Parametric modeling → Point cloud conversion |

### Technical Pipeline 
```mermaid
graph LR
    A[Real-world Capture] -->|Photogrammetry| B(scene4_tree.ply)
    C[CAD Design] -->|SolidWorks| D[Assembly.sldasm]
    D -->|CloudWorks| E(scene1, 2, 3*.ply)
    B & E --> F[OMPL Planning Environment]
