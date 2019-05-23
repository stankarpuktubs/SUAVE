
## @ingroup Methods-Geometry-Two_Dimensional-Planform
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

## @ingroup Methods-Geometry-Two_Dimensional-Planform
def cabin_planform(cabin):
    
    s_width         = cabin.seat_width
    s_pitch         = cabin.seat_pitch
    aisle_width     = cabin.aisle_width
    n_aisles        = cabin.n_aisles
    cabin_length    = cabin.get_total_length()
    i_start_segment = cabin.i_start_segment
    i_end_segment   = cabin.i_end_segment
    
    n_sections = i_end_segment - i_start_segment + 1
    x_list, w_list = np.empty(n_sections), np.empty(n_sections)
    for i in range(n_sections):
        x_list[i] = cabin.Segments[i + i_start_segment].percent_x_location*cabin_length
        w_list[i] = cabin.Segments[i + i_start_segment].width
        

    x_full_list                 = np.linspace(x_list[0], x_list[-1], 1000)
    w_full_list                 = np.interp(x_full_list, x_list, w_list)
    min_feasible_x_loc_list     = np.where(w_full_list > (s_width + n_aisles*aisle_width))[0]
    i_start                     = min_feasible_x_loc_list[0]
    i_end                       = min_feasible_x_loc_list[-1]
    x_start                     = x_full_list[i_start]
    x_end                       = x_full_list[i_end]
    
    n_steps     = np.floor((x_end-x_start)/s_pitch)
    x           = np.linspace(x_start, x_start+n_steps*s_pitch, int(n_steps))
    w           = np.interp(x, x_list, w_list)
    n_list      = np.zeros_like(x)

    n_list      = np.floor((w-n_aisles*aisle_width)/(s_width))
        
    total_seats      = sum(n_list)
    total_seat_area  = np.trapz(w_full_list[i_start:i_end+1], x_full_list[i_start:i_end+1])
    total_cargo_area = np.trapz(w_list, x_list)

    cabin.total_seats       = total_seats
    cabin.total_seat_area   = total_seat_area
    cabin.total_cargo_area  = total_cargo_area
    cabin.sections          = n_list

    return 0