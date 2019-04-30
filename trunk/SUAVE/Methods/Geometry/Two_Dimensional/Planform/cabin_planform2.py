
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
def cabin_planform2(cabin):
    
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
        w_list.append(segment.width/2)

    x_list = np.array(x_list)
    w_list = np.array(w_list)

    x = np.linspace(x_list[0],x_list[-1],1000)
    w = np.interp(x, x_list, w_list)
    n_list = zeros_like(x)

    for i in range(len(n_options)):
        n_list[w > width_list[i] and w < width_list[i+1] = n_options[i]
        


            
    cabin.total_seats = total_seats
    cabin.total_seat_area = total_seat_area
    cabin.total_cargo_area = total_cargo_area
    cabin.sections = n_rows_n_seats
    
    return 0