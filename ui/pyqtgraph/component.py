
class Component():
    """
    Abstract representation of a PyQtGraph component, exposing only necessary methods
    """

    def set_pos(self, x_pos, y_pos):
        pass

    def hide(self):
        self.get_item().hide()

    def show(self):
        self.get_item().show()

    def get_item(self):
        pass