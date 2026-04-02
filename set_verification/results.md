# Results of set verification
## Control Invariance CIS
I sampled 100'000 states at 0.995 * max_vel_predicted in order to be almost on the border. Using 1 instead of 0.995 resulted in some states slighlty outside the set.
Of these 100'000 states, 719 were not able to return in the set in one step. To avoid long computation times, I performed the other tests on these 719 states that failed.

## Within-N-step CIS
Of the 719 failing states, 200 returned for at least one horizon in [2,45]. So 28% of times for a state that is not CIS it is Within-N-step CIS.

## Up-to-N-step CIS 
Regarding the analysis for the Up-to-N-step CIS, the horizon with more successes is 24, with 162 returns. All the other horizons are reported in the image below. In any case, also with horizon 2, the return rate is higher than 1.
![Successful states per horizon](data_results/successful_states_per_horizon_ipopt.png)

## Back-and-forth-N-step Control Invariance

