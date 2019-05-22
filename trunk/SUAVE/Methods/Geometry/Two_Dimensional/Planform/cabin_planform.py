
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
    s_width         = cabin.seat_width
    s_pitch         = cabin.seat_pitch
    aisle_width     = cabin.aisle_width
    n_aisles        = cabin.n_aisles
    cabin_length    = cabin.get_total_length()
    
    x_list, w_list = np.empty(len(cabin.Segments)), np.empty(len(cabin.Segments))
    for i, segment in enumerate(cabin.Segments):
        x_list[i] = segment.percent_x_location*cabin_length
        w_list[i] = segment.width

    x_full_list                 = np.linspace(x_list[0], x_list[-1], 1000)
    w_full_list                 = np.interp(x_full_list, x_list, w_list)
    min_feasible_x_loc_list     = np.where(w_full_list > (s_width + n_aisles*aisle_width)[0]
    x_start                     = x_list[min_feasible_x_loc_list[0]]
    x_end                       = x_list[min_feasible_x_loc_list[-1]]
    
    n_steps     = np.floor((x_end-x_start)/s_pitch)
    x           = np.linspace(x_start, x_start+n_steps*s_pitch, int(n_steps))
    w           = np.interp(x, x_list, w_list)
    n_list      = np.zeros_like(x)

    n_list      = np.floor((w-n_aisles*aisle_width)/(s_width))
        
    total_seats      = sum(n_list)
    total_seat_area  = np.trapz(w, x)
    total_cargo_area = np.trapz(w_list, x_list)

    cabin.total_seats       = total_seats
    cabin.total_seat_area   = total_seat_area
    cabin.total_cargo_area  = total_cargo_area
    cabin.sections          = n_list

    return 0