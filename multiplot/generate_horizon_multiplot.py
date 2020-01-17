#!/usr/bin/env python

# functions  / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
def write_line(file, str):
    file.write(str + newline .encode('utf8'))

def write_open_tag(file, num_tab, str):
    file.write(tab*num_tab + larrow + str + rarrow + newline .encode('utf8'))

def write_close_tag(file, num_tab, str):
    file.write(tab*num_tab + lsarrow + str + rarrow + newline .encode('utf8'))

def write_single_line_tag(file, num_tab, tag, val):
    file.write(tab*num_tab + larrow + tag + rarrow + val + lsarrow + tag + rarrow + newline .encode('utf8'))

# main / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

file = open("soft_objectives.xml", "w")

# plot information
background_color_val = "#ffffff"
foreground_color_val = "#000000"
topic_list = ["/nmpc/soft_obj/aoa", "/nmpc/soft_obj/h", "/nmpc/soft_obj/r"] # corresponds to the row-wise plots we will display
topic_type = "std_msgs/Float32MultiArray"
num_rows = len(topic_list)
num_cols = 1 # no multi column functionality currently in this script
num_curves = 51 # horizon length + 1
# titles
row_titles = ["Soft AoA","Soft Height","Soft Radial Prox."]
# x axis
x_axis_title = "Nodes in Horizon"
x_axis_title_visible = "true"
x_axis_title_type = "1"
x_axis_field_name = "data/"
x_axis_field_type = "0"
# y axis
y_axis_title = "Objective Value"
y_axis_title_visible = "true"
y_axis_title_type = "1"
y_axis_field_name = "data/"
y_axis_field_type = "0"
# data
custom_plot_color_val = "#ef2929"
circular_buffer_capacity_val = "1" # not sure if used
time_frame_length_val = "0.1" # best set to NMPC iteration timestep
data_type = "2" # circluar buffer 
# style
lines_interpolate_val = "false"
pen_style_val = "1" # solid line
pen_width_val = "5"
render_antialias_val = "false"
steps_invert_val = "false"
sticks_baseline_val = "0"
sticks_orientation_val = "2" # vertical
style_type = "1" # sticks
# misc
subscriber_queue_size_val = "1"
# legend
visible_val = "false"
# other
plot_rate_val = "30" # Hz

# DONT TOUCH BELOW UNLESS YOU KNOW WHAT YOU ARE DOING  / / / / / / / / / / / / /

# formatting
tab = "    "
newline = "\n"
larrow = "<"
rarrow = ">"
lsarrow = "</"

# tags
rqt_multiplot = "rqt_multiplot"
table = "table"
background_color = "background_color"
foreground_color = "foreground_color"
plots = "plots"
row_i = "row_"
column_i = "column_"
axes = "axes"
x_axis = "x_axis"
y_axis = "y_axis"
curves = "curves"
curve_i = "curve_"
scale = "scale"
data = "data"
color = "color"
style = "style"
custom_title = "custom_title"
title_type = "title_type"
title_visible = "title_visible"
field = "field"
field_type = "field_type"
absolute_maximum = "absolute_maximum"
absolute_minimum = "absolute_minimum"
relative_maximum = "relative_maximum"
relative_minimum = "relative_minimum"
type = "type"
topic = "topic"
custom_color = "custom_color"
circular_buffer_capacity = "circular_buffer_capacity"
time_frame_length = "time_frame_length"
lines_interpolate = "lines_interpolate"
pen_style = "pen_style"
pen_width = "pen_width"
render_antialias = "render_antialias"
steps_invert = "steps_invert"
sticks_baseline = "sticks_baseline"
sticks_orientation = "sticks_orientation"
subscriber_queue_size = "subscriber_queue_size"
legend = "legend"
visible = "visible"
plot_rate = "plot_rate"
title = "title"

# full lines
header_line = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
link_cursor_line = "<link_cursor>false</link_cursor>"
link_scale_line = "<link_scale>false</link_scale>"
track_points_line = "<track_points>false</track_points>"

# write the lines! / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
write_line(file, header_line)
write_open_tag(file, 0, rqt_multiplot)
write_open_tag(file, 1, table)
write_single_line_tag(file, 2, background_color, background_color_val)
write_single_line_tag(file, 2, foreground_color, foreground_color_val)
write_line(file, tab*2 + link_cursor_line)
write_line(file, tab*2 + link_scale_line)
write_open_tag(file, 2, plots)

for i_row in range(num_rows):
    # START rows
    write_open_tag(file, 3, row_i + str(i_row))

    # START columns
    write_open_tag(file, 4, column_i + "0") # for now only considering 1 column
    write_open_tag(file, 5, axes)
    write_open_tag(file, 6, axes)

    # START x axis
    write_open_tag(file, 7, x_axis)
    write_single_line_tag(file, 8, custom_title, x_axis_title)
    write_single_line_tag(file, 8, title_type, x_axis_title_type)
    write_single_line_tag(file, 8, title_visible, x_axis_title_visible)
    write_close_tag(file, 7, x_axis)
    # END x axis

    # START y axis
    write_open_tag(file, 7, y_axis)
    write_single_line_tag(file, 8, custom_title, y_axis_title)
    write_single_line_tag(file, 8, title_type, y_axis_title_type)
    write_single_line_tag(file, 8, title_visible, y_axis_title_visible)
    write_close_tag(file, 7, y_axis)
    # END y axis

    write_close_tag(file, 6, axes)
    write_close_tag(file, 5, axes)

    # START curves
    write_open_tag(file, 5, curves)
    for i_curv in range(num_curves):
        # START curve i
        write_open_tag(file, 6, curve_i + str(i_curv))
        write_open_tag(file, 7, axes)

        # START x axis
        write_open_tag(file, 8, x_axis)
        write_single_line_tag(file, 9, field, x_axis_field_name + str(i_curv+51))
        write_single_line_tag(file, 9, field_type, x_axis_field_type)
        write_open_tag(file, 9, scale)
        write_line(file, tab*10 + "<absolute_maximum>1000</absolute_maximum>") # not used currently because "auto" setting
        write_line(file, tab*10 + "<absolute_minimum>0</absolute_minimum>") # not used currently because "auto" setting
        write_line(file, tab*10 + "<relative_maximum>0</relative_maximum>") # not used currently because "auto" setting
        write_line(file, tab*10 + "<relative_minimum>-1000</relative_minimum>") # not used currently because "auto" setting
        write_single_line_tag(file, 10, type, "0") # auto
        write_close_tag(file, 9, scale)
        write_single_line_tag(file, 9, topic, topic_list[i_row])
        write_single_line_tag(file, 9, type, topic_type)
        write_close_tag(file, 8, x_axis)
        # END x axis

        # START y axis
        write_open_tag(file, 8, y_axis)
        write_single_line_tag(file, 9, field, y_axis_field_name + str(i_curv))
        write_single_line_tag(file, 9, field_type, y_axis_field_type)
        write_open_tag(file, 9, scale)
        write_line(file, tab*10 + "<absolute_maximum>1000</absolute_maximum>") # not used currently because "auto" setting
        write_line(file, tab*10 + "<absolute_minimum>0</absolute_minimum>") # not used currently because "auto" setting
        write_line(file, tab*10 + "<relative_maximum>0</relative_maximum>") # not used currently because "auto" setting
        write_line(file, tab*10 + "<relative_minimum>-1000</relative_minimum>") # not used currently because "auto" setting
        write_single_line_tag(file, 10, type, "0") # auto
        write_close_tag(file, 9, scale)
        write_single_line_tag(file, 9, topic, topic_list[i_row])
        write_single_line_tag(file, 9, type, topic_type)
        write_close_tag(file, 8, y_axis)
        # END y axis

        write_close_tag(file, 7, axes)

        # START color
        write_open_tag(file, 7, color)
        write_single_line_tag(file, 8, custom_color, custom_plot_color_val)
        write_single_line_tag(file, 8, type, "1") # custom
        write_close_tag(file, 7, color)
        # END color

        # START data
        write_open_tag(file, 7, data)
        write_single_line_tag(file, 8, circular_buffer_capacity, circular_buffer_capacity_val)
        write_single_line_tag(file, 8, time_frame_length, time_frame_length_val)
        write_single_line_tag(file, 8, type, data_type)
        write_close_tag(file, 7, data)
        # END data

        # START style
        write_open_tag(file, 7, style)
        write_single_line_tag(file, 8, lines_interpolate, lines_interpolate_val)
        write_single_line_tag(file, 8, pen_style, pen_style_val)
        write_single_line_tag(file, 8, pen_width, pen_width_val)
        write_single_line_tag(file, 8, render_antialias, render_antialias_val)
        write_single_line_tag(file, 8, steps_invert, steps_invert_val)
        write_single_line_tag(file, 8, sticks_baseline, sticks_baseline_val)
        write_single_line_tag(file, 8, sticks_orientation, sticks_orientation_val)
        write_single_line_tag(file, 8, type, style_type)
        write_close_tag(file, 7, style)
        # END style

        write_single_line_tag(file, 7, subscriber_queue_size, subscriber_queue_size_val)
        write_single_line_tag(file, 7, title, str(i_curv))
        write_close_tag(file, 6, curve_i + str(i_curv))
        # END curve i

    write_close_tag(file, 5, curves)
    # END curves

    # START legend
    write_open_tag(file, 5, legend)
    write_single_line_tag(file, 6, visible, visible_val)
    write_close_tag(file, 5, legend)
    # END legend

    write_single_line_tag(file, 5, plot_rate, plot_rate_val)
    write_single_line_tag(file, 5, title, row_titles[i_row])
    write_close_tag(file, 4, column_i + "0") # for now only considering 1 column
    # END columns

    write_close_tag(file, 3, row_i + str(i_row))
    # END rows

write_close_tag(file, 2, plots)
write_line(file, tab*2 + track_points_line)
write_close_tag(file, 1, table)
write_close_tag(file, 0, rqt_multiplot)

file.close()
