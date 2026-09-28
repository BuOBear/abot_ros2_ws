"""Reuse the actual mecanum base without the obsolete fixed camera transform."""
import xml.etree.ElementTree as ET
import xacro


def base_description(filename):
    root = ET.fromstring(xacro.process_file(str(filename)).toxml())
    removed_links = {'camera_link', 'camera_optical_frame'}
    for element in list(root):
        if element.tag == 'link' and element.get('name') in removed_links:
            root.remove(element)
        elif element.tag == 'joint':
            parent, child = element.find('parent'), element.find('child')
            if any(part is not None and part.get('link') in removed_links
                   for part in (parent, child)):
                root.remove(element)
    root.set('name', 'jy_physical_base')
    return ET.tostring(root, encoding='unicode')
