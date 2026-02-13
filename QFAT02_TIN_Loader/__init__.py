# -*- coding: utf-8 -*-
def classFactory(iface):
    from .qfat02_loader import Qfat02LoaderPlugin
    return Qfat02LoaderPlugin(iface)
