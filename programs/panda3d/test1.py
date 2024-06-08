from panda3d_viewer import Viewer, ViewerConfig

config = ViewerConfig()
config.set_window_size(800, 600)
config.enable_antialiasing(True, multisamples=4)

config.enable_shadow(True)
config.show_axes(False)
config.show_grid(False)
# config.show_floor(True)

# config.enable_antialiasing(True, multisamples = 4)

mm_to_unit = 0.01

with Viewer(window_type='onscreen', window_title='example', config=config) as viewer:

    viewer.append_group('root')
    # viewer.set_background_color((0, 0.5, 0, 1.0))
    # viewer.append_box('root', 'box_node', size=(1, 1, 1))
    # viewer.append_sphere('root', 'sphere_node', radius=0.5)

    # viewer.set_material('root', 'box_node', color_rgba=(0.7, 0.1, 0.1, 1))
    # viewer.set_material('root', 'sphere_node', color_rgba=(0.1, 0.7, 0.1, 1))

    for i in range(5):
        # viewer.append_mesh('root', 'mesh'+str(i), 'rr_bar_plate.stl')
        viewer.append_mesh('root', 'mesh1', 'base_plate.stl', scale={mm_to_unit, mm_to_unit, mm_to_unit})
        viewer.set_material('root', 'mesh1', color_rgba=(0.3, 0.3, 0.1, 1))
        viewer.append_mesh('root', 'mesh2', 'move_base_psd_holder.stl', scale={mm_to_unit, mm_to_unit, mm_to_unit})
        viewer.set_material('root', 'mesh2', color_rgba=(0, 0.5, 0, 1.0))


    # viewer.move_nodes('root', {
    #     'box_node': ((0, 0, 0.5), (1, 0, 0, 0)),
    #     'sphere_node': ((0, 0, 1.5), (1, 0, 0, 0))})

    for i in range(5):
        viewer.move_nodes('root', {'mesh1': ((100 * mm_to_unit, 0, 20 * mm_to_unit), (1, 0, 0, 0))})
        viewer.move_nodes('root', {'mesh2': ((0, 0, 0), (0.7, 0.7, 0, 0))})

    # viewer.reset_camera(pos=(500 * mm_to_unit, -50 * mm_to_unit, 1000 * mm_to_unit), look_at=(0, 0, 10 * mm_to_unit))
    viewer.reset_camera(pos=(150 * mm_to_unit, -50 * mm_to_unit, 120 * mm_to_unit), look_at=(0, 0, 10 * mm_to_unit))
    viewer.save_screenshot(filename='box_and_sphere.png')
