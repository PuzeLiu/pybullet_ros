
class PostProcessor:
    def __init__(self, pybullet, models_dict):
        self.pb = pybullet
        self.models = models_dict

    def load(self):
        """
        Load you own post process, such as disable collision
        """
        pass