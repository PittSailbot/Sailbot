def singleton(cls):
    """A decorator which prevents duplicate classes from being created.
    Useful for physical objects where only one exists.
        - Import, then invoke use @singleton before class definition"""
    instances = {}

    def get_instance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance
