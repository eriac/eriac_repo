from bokeh.plotting import figure, output_file, show, reset_output
import numpy as np

x=np.linspace(0,10,100)
y1=np.sin(x)
y2=np.cos(x)

reset_output()

output_file("graph.html")

TOOLTIPS = [
    ("index", "$index"),
    ("(x,y)", "($x, $y)"),
]

p = figure(tooltips=TOOLTIPS, title="sin, cos", x_axis_label="x", y_axis_label="y")

p.line(x, y1, legend="sin")
p.line(x, y2, legend="cos")

p.legend.click_policy = "hide"

show(p)
