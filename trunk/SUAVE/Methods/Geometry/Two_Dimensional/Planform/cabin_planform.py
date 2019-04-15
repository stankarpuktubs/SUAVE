
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
    cabin_length = cabin.lengths.total
    
    x_list = []
    w_list = []
    for segment in cabin.Segments:
        x_list.append(segment.percent_x_location*cabin_length)
        w_list.append(segment.width/2)
    
    # Generates linear fits between the x,w points defining the fuselage
    fit_list, x_subset_list, w_subset_list = [], [], []
    for i in range(1,len(x)):
        x_subset = [x[i-1],x[i]]
        x_subset_list.append(x_subset)
        w_subset = [w[i-1],w[i]]
        w_subset_list.append(w_subset)
        fit_list.append(np.polyfit(w_subset,x_subset,1))
        
    # Finds the x locations where the aircraft is the specified width
    x_list, w_list = [], []
    x_cargo_list, w_cargo_list = [], []
    for width in width_list:
        for i in range(len(w_subset_list)):
            subset = w_subset_list[i]
            # Assumes the width's inputted are half of what they should be
            if width <= 2*max(subset) and width >= 2*min(subset):
                x_calc = np.polyval(fit_list[i], width/2)
                x_list.append(x_calc)
                w_list.append(width)

    # Sorts the lists by the x location
    x_list, w_list = zip(*sorted(zip(x_list, w_list)))
    x_list, w_list = list(x_list), list(w_list)
    
    # Eliminates points that are effectively the same (same x,w)
    dup_index = []
    for i in range(len(x_list)-1):
        if x_list[i+1] - x_list[i] <= 2:
            dup_index.append(i)

    for index in dup_index:
        x_list.pop(index)
        w_list.pop(index)
        
    # Calculate out how many rows of each number of seats
    n_rows = []
    n_rows_n_seats = []
    n_seats = []
    l = min(x_list)
    index = x_list.index(l)
    current_w = w_list[index]
    n = 0
    i_current = 0
    if len(w_list) >= 3:
        while i_current < len(w_list)-2:
            if l in x_list:
                i_current = x_list.index(l)
                i_new = i_current + 1
                delta_x = x_list[i_new]-x_list[i_current]
                w_new = w_list[i_new]
            else:
                i_current += 1
                i_new += 1
                delta_x = x_list[i_new]-l
                while delta_x < s_pitch:
                    delta_x = x_list[i_new]-l 
                    i_current += 1
                    i_new += 1
                
            if w_list[i_new] >= w_list[i_current]:
                n_row = np.ceil(delta_x/s_pitch)
                n_rows.append((n_row,w_list[i_current]))
                n_rows_n_seats.append(n_row,(w_list[i_current]-aisle_width)/s_width)
                
            elif w_list[i_new] < w_list[i_current]:
                n_row = np.floor(delta_x/s_pitch)
                n_rows.append((n_row,w_list[i_new]))
                n_rows_n_seats.append(n_row,(w_list[i_new]-aisle_width)/s_width)
            
            l += n_row*s_pitch
    else:
        n_rows.append((np.floor((x_list[-1]-x_list[0])/s_pitch),w_list[0]))
        n_rows_n_seats.append((np.floor((x_list[-1]-x_list[0])/s_pitch),(w_list[0]-aisle_width)/s_width))
        
    # Counts the total number of seats
    total_seats, total_seat_area = 0, 0
    for elem in n_rows:
        total_seats += elem[0]*(elem[1]-aisle_width)/s_width
        total_seat_area += elem[0]*elem[1]*s_pitch
    
    total_cargo_area = 0
    for i in range(1,len(x)):
        if w[i-1] <= w[i]:
            total_cargo_area += 2*(x[i]-x[i-1])*w[i-1]+(w[i]-w[i-1])*(x[i]-x[i-1])
        else:
            total_cargo_area += 2*(x[i]-x[i-1])*w[i]+(w[i-1]-w[i])*(x[i]-x[i-1])
            
    cabin.total_seats = total_seats
    cabin.total_seat_area = total_seat_area
    cabin.total_cargo_area = total_cargo_area
    cabin.sections = n_rows_n_seats
    
    return 0