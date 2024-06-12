## Overview of POC Outcomes

1. **Minimization of Total Distance:**
   - The algorithm defaults to minimizing the total distance covered. However, for engineers exceeding the required distance, it prioritizes adherence to slot timings, aiming to reach the CX fastest within the given time slot. This may result in overuse of engineers.

2. **Adding Penalty for Extra Engineers:**
   - To address the overuse of engineers, a penalty is added to the addition of engineers. This signals the algorithm to add an extra engineer only when it is not feasible to adhere to both time constraints and distance covered by the current number of engineers.

3. **Exploration of Solution Algorithms:**
   - Investigate "First Solution Algorithms" and "Search Heuristic Algorithms" to determine the best fit for our scenario. While "GUIDED_LOCAL_SEARCH" appears most suitable based on current implementations and Google developers of the package (as indicated in comments), thorough research on all available options is necessary.

4. **Penalties for Drop Locations:**
   - Apply high penalties to location drops to ensure that locations are not dropped when engineers can visit them.

5. **Variable Start Positions of Engineers:**
   - Implement variable start positions for engineers to enhance flexibility and optimization of routes.

6. **Incorporating Service Time:**
   - Incorporate service time into the time matrix. Directly adding service time to travel time may lead to slot adherence considering both travel and service time. Instead, add service time to the next path: \( time(j) = travel(i \rightarrow j) + service(i) \).

