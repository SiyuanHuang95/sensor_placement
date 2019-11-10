For every new generated sensor, we calculate two parameters:

1. $\tau_a$: newly covered dangerous zone points
2. $\tau_b$: overlapped dangerous zone points with already existed accepted sensors.



if $\tau_a$ < Threshold:

​	too few points are covered, discard.

else:

​	if ($\frac{\tau_a}{\tau_b} > Threshold_a$):

​			accept as a new sensor

  	else:

​			$\varepsilon = Threshold_a - \frac{\tau_a}{\tau_b} $

​			accept with probability: $\frac{1}{exp(\frac{\varepsilon}{Threshold_a})+1}$

Generate new sensor at nearby position:

$x = (exp(\varepsilon) )*c*x$

$c$ describe the search length in the neighbor.