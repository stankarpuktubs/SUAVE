
## @ingroup Methods-Geometry-Two_Dimensional-Cross_Section-Planform
# cabin_planform.py
#
# Created:  April 2019, M. Dethy

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np
# ----------------------------------------------------------------------
#  Methods
# ----------------------------------------------------------------------

## @ingroup Methods-Geometry-Two_Dimensional-Cross_Section-Planform
def cabin_planform(cabin):
    
    # Declared as constants for now, can be made more general variables later
    s_width = cabin.seat_width
    s_pitch = cabin.seat_pitch
    aisle_width = cabin.aisle_width
    n_options = cabin.seats_abreast_options
    width_list = [n*s_width+aisle_width for n in n_options]
    width_list.append(width_list[-1] + s_width)
    cabin_length = cabin.lengths.total
    
    x_list = []
    w_list = []
    for segment in cabin.Segments:
        x_list.append(segment.percent_x_location*cabin_length)
        w_list.append(segment.width)

    x_list = np.array(x_list)
    w_list = np.array(w_list)

    min_feasible_x = np.where(x_list > min(width_list))[0]
    x_start = x_list[min_feasible_x[0]]
    x_end = x_list[min_feasible_x[-1]]
    
    n_steps = round((x_end-x_start)/s_pitch)
    x = np.linspace(x_start, x_start+n_steps*s_pitch, int(n_steps))
    w = np.interp(x, x_list, w_list)
    n_list = np.zeros_like(x)

    for i in range(len(n_options)):
        n_list[w >= width_list[i]] = n_options[i]
        
    total_seats = sum(n_list)
    avg_base = (n_list[:-1] * s_pitch + 20 + n_list[1:] * s_pitch + 20)*0.5
    total_seat_area = sum(avg_base*38)

    cabin.total_seats = total_seats
    cabin.total_seat_area = total_seat_area
    cabin.sections = n_list

    return 0