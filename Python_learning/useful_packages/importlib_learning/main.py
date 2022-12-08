'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-11-30 15:33:01
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

from easydict import EasyDict
import importlib


def get_obj_from_str(string, reload=False):
    """Borrowed from taming-transformer

    Args:
        string (_type_): _description_
        reload (bool, optional): _description_. Defaults to False.

    Returns:
        _type_: _description_
    """
    module, cls = string.rsplit(".", 1)
    print(module, cls)
    if reload:
        module_imp = importlib.import_module(module)
        importlib.reload(module_imp)
    return getattr(importlib.import_module(module, package=None), cls)


def instantiate_from_config(config):
    if not "target" in config:
        raise KeyError("Expected key `target` to instantiate.")
    return get_obj_from_str(config["target"])(**config.get("params", dict()))


def instantiate_from_config(config, params=None):
    if not "target" in config:
        raise KeyError("Expected key `target` to instantiate.")
    if params is None:
        return get_obj_from_str(config["target"])(**config.get("params", dict()))
    else:
        return get_obj_from_str(config["target"])(**params, **config.get("params", dict()))


def load_custom_module():
    config = EasyDict(target="models.module1.Model1")
    config = EasyDict(target="dataset.PU1K")

    my_model = instantiate_from_config(config)
    print(type(my_model))


def load_installed_package():
    config = EasyDict(target="torch.optim.Adam", params={'lr': 0.001})

    optimizer = instantiate_from_config(config)
    print(type(optimizer))


if __name__ == "__main__":
    load_installed_package()
