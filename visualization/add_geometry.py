import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import platform
import random
import threading
import time

class SpheresApp:
    MENU_SPHERE = 1
    MENU_RANDOM = 2
    MENU_QUIT = 3

    def __init__(self):
        self._id = 0
        self.window = gui.Application.instance.create_window("Add Spheres Example", 1024, 768)
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.scene.scene.set_background([1, 1, 1, 1])
        self.scene.scene.scene.set_sun_light([-1, -1, -1], [1, 1, 1], 100000) 
        self.scene.scene.scene.enable_sun_light(True)
        bbox = o3d.geometry.AxisAlignedBoundingBox([-10, -10, -10], [10, 10, 10])
        self.scene.setup_camera(60, bbox, [0, 0, 0])

        push_button = gui.Button('...')        
        self.window.add_child(self.scene)
        self.window.add_child(push_button)

        if gui.Application.instance.menubar is None:
            debug_menu = gui.Menu()
            debug_menu.add_item("Add Sphere", SpheresApp.MENU_SPHERE)
            debug_menu.add_item("Add Random Spheres", SpheresApp.MENU_RANDOM)

            menu = gui.Menu()
            menu.add_menu("Debug", debug_menu)
            gui.Application.instance.menubar = menu

        self.window.set_on_menu_item_activated(SpheresApp.MENU_SPHERE, self._on_menu_sphere)
        self.window.set_on_menu_item_activated(SpheresApp.MENU_RANDOM, self._on_menu_random)
        self.window.set_on_menu_item_activated(SpheresApp.MENU_QUIT, self._on_menu_quit) 
        # self.window.button.set_on_clicked(self.on_menu_sphere_)

    def add_sphere(self):
        self._id += 1
        mat = rendering.MaterialRecord()
        mat.base_color = [random.random(), random.random(), random.random(), 1.0]
        mat.shader = "defaultLit"
        sphere = o3d.geometry.TriangleMesh.create_sphere(0.5)
        sphere.compute_vertex_normals()
        sphere.translate([
            10.0 * random.uniform(-1.0, 1.0), 10.0 * random.uniform(-1.0, 1.0),
            10.0 * random.uniform(-1.0, 1.0)
        ])
        self.scene.scene.add_geometry("sphere" + str(self._id), sphere, mat)

    def _on_menu_sphere(self):
        self.add_sphere()

    def _on_menu_random(self):
        def thread_main():
            for _ in range(0, 20):
                gui.Application.instance.post_to_main_thread(self.window, self.add_sphere)
                time.sleep(1)
        threading.Thread(target=thread_main).start()

    def _on_menu_quit(self):
        gui.Application.instance.quit()


def main():
    gui.Application.instance.initialize()
    SpheresApp()
    gui.Application.instance.run()


if __name__ == "__main__":
    main()
