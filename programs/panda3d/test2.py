from math import cos, sin, pi
import imageio # install imageio: pip install imageio

from panda3d_viewer import Viewer, ViewerConfig

config = ViewerConfig()
config.set_window_size(800, 600)
#config.enable_antialiasing(True, multisamples=4)
config.enable_shadow(True)
config.show_axes(False)
config.show_grid(False)
config.show_floor(True)

viewer = Viewer(window_type='offscreen', config=config)

viewer.append_group('root')
#viewer.append_sphere('root', 'sphere_node', radius=0.5)
viewer.append_mesh('root', 'sphere_node', 'rr_bar_plate.stl', scale={0.05, 0.05, 0.05})
viewer.set_material('root', 'sphere_node', color_rgba=(0.1, 0.7, 0.1, 1))

viewer.append_mesh('root', 'sphere_node2', 'rr_bar_plate.stl', scale={0.05, 0.05, 0.05})
viewer.set_material('root', 'sphere_node2', color_rgba=(0.1, 0.7, 0.1, 1))

with imageio.get_writer('sphere_anim.gif', mode='I') as writer:
    for i in range(50):
        angle = 2 * pi * i / 50
        x = 4 * cos(angle)
        y = 4 * sin(angle)
        z = 0.5 + 0.5 * abs(sin(angle))

        viewer.move_nodes('root', {'sphere_node': ((0, 0, z), (1, 0, 0, 0))})
        viewer.move_nodes('root', {'sphere_node2': ((0.5, 0, 1), (1, 0, 0, 0))})
        viewer.reset_camera(pos=(x, y, 2), look_at=(0, 0, 1))

        image_rgb = viewer.get_screenshot(requested_format='RGB')
        writer.append_data(image_rgb)
