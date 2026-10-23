## IST4 - Policies and Guidelines 

Most scientific and research organizations require policies and 
guidelines to inform their partners about how to contribute to the overall organization.

This document represents the collective wisdom of the Intelligent Systems for... (IST4) group. 
It outlines how you can contribute your projects and ideas to our shared structure.

# Always provide a README.md

All projects will have variability. This guideline acknowledges that your project may have 
unique objectives requiring deviations from the structure presented here. 
Therefore, it is essential that your project includes a README.md file explaining how to interact with it.

Your README.md should contain a minimal overview of your code, 
enabling third parties to quickly understand and use your contributions. 
Avoid unnecessary embellishments, focus on clarity and functionality over aesthetics.

# Code and algorithm breaking behaviour

Throughout your development cycle, you will write algorithms that are constantly evolving, 
especially when exploring new fields of engineering. It is understandable that your project may not always work as intended.

To avoid confusion:

* Use the main branch as your stable baseline.
* When making significant changes, create a new branch with a descriptive name (e.g., 3D-convolutions).
* Once your code is stable and thoroughly tested, merge the experimental branch back into main.

```
---> main branch
        |
        |
        V
    -> 3D convolutions ->
```

After stabilization:

```
---> main branch ---------------> merged main
        |                       ^
        |                       |
        V                       |
    -> 3D convolutions ---------> 
```

This approach ensures:

* The main branch always remains in a working state for external users.
* Experimental changes are isolated until validated.

# Project layout

Most projects can be seperated into the following structure:

```
--> /libraries/
--> /tests/
--> /examples/
--> /data/
--> ReadMe.md
```

where:
* /libraries/: Contains core algorithms with clean, modular interfaces that should work without modifications. Ensure your functions are intuitive and well-documented.

* /tests/: Includes small tests demonstrating that your algorithm works in basic scenarios. This helps others verify functionality and understand how to use your interfaces.

* /examples/: Contains scripts showcasing how *all* parameters in your interface should be used. For example, explain why a threshold of 1 might work in some cases but not others, and how the algorithm behaves under different conditions.

* /data/: Stores all necessary datasets for the project. If your dataset is too large, contact the repository maintainer for alternative solutions.

