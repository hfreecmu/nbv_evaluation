#!/usr/bin/env python3

import rospy
from custom_msgs.srv import GazeboModel
import os
import xml.etree.ElementTree as ET

skip_model_ids = ['sun',
                  'dirt',
                  'Apriltag',
                  'tree']

class GazeboModelHandler:
    def __init__(self, link_dict):
        self.link_dict = link_dict

    def handle_request(self, req):
        link_id = req.link_id.data

        if self.link_dict.get(link_id) is None:
            rospy.logwarn(link_id + ' not found in link_dict')

            return [0, 0, 0], False
        
        scale = self.link_dict[link_id]['scale']
        return scale, True


def search_single(node, tag_id, throw_error, sdf_path):
    res = None
    for child in node:
        if child.tag == tag_id:
            if res is not None:
                if throw_error:
                    raise RuntimeError(tag_id + ' found twice in ' + sdf_path)
                else:
                    return None
            else:
                res = child
    
    if res is None:
        if throw_error:
            raise RuntimeError(tag_id + ' not found in ' + sdf_path)
        else:
            return None

    return res

def parse_link(link, sdf_path, model_prefixes, link_dict):
    #right now we are just getting name and scale info
    #we can add other details later
    link_name = link.attrib['name']
    
    visual = search_single(link, 'visual', True, sdf_path)
    geometry = search_single(visual, 'geometry', True, sdf_path)
    mesh = search_single(geometry, 'mesh', False, sdf_path)
    if mesh is None:
        return
    scale = search_single(mesh, 'scale', True, sdf_path)

    model_prefixes.append(link_name)
    link_id = '::'.join(model_prefixes)

    if link_dict.get(link_id) is not None:
        raise RuntimeError('link_id appears twice: ' + link_id)
    
    scale = scale.text.split(' ')
    if len(scale) != 3:
        raise RuntimeError('scale len not 3 for ' + link_id + ' in ' + sdf_path)

    for i in range(len(scale)):
        scale[i] = float(scale[i])

    link_dict[link_id] = {'scale': scale}
        
def parse_model(model, sdf_path, gazebo_model_path, model_prefixes, model_name, link_dict):
    if (model_name is None):
        model_name = model.attrib['name']
        
    model_prefixes.append(model_name)

    for child in model:
        if child.tag == 'model':
            raise RuntimeError('I know I wrote to support this but why is it here?')
            parse_model(child, sdf_path, gazebo_model_path, model_prefixes.copy(), None, link_dict)
        elif child.tag == 'link':
            parse_link(child, sdf_path, model_prefixes.copy(), link_dict)
        elif child.tag == 'include':
            #raise RuntimeError('I know I wrote to support this but why is it here?')
            parse_include(child, sdf_path, gazebo_model_path, model_prefixes.copy(), link_dict)
        else:
            continue

def parse_sdf(sdf_path, gazebo_model_path, model_prefixes, model_name, link_dict):
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    model = search_single(root, 'model', False, sdf_path)

    if model != None:
        parse_model(model, sdf_path, gazebo_model_path, model_prefixes, model_name, link_dict)

def parse_include(include, sdf_path, gazebo_model_path, model_prefixes, link_dict, skip_model_ids=[]):
    uri = search_single(include, 'uri', True, sdf_path).text
    
    for skip_id in skip_model_ids:
        if skip_id in uri:
            return
        
    #TODO adding this for exp, take out if we want to parse further
    if not ('cluster' in uri):
        return

    model_name = search_single(include, 'name', False, sdf_path)

    if model_name != None:
        model_name = model_name.text

    #model
    if uri.startswith('model://'):
        #TODO make exp_clusters better
        path_suffix = uri.split('model://')[1]
        # model_dir = os.path.join(gazebo_model_path, 'exp_clusters', path_suffix)
        model_dir = os.path.join(gazebo_model_path, path_suffix)
    else:
        model_dir = uri

    model_path = os.path.join(model_dir, 'model.sdf')

    if not os.path.exists(model_path):
        raise RuntimeError('model path does not exist: ' + model_path + ', ' + sdf_path)
        #print('model path does not exist: ' + model_path + ', ' + sdf_path)
        #return

    parse_sdf(model_path, gazebo_model_path, model_prefixes, model_name, link_dict)    

def parse_world(world_path, gazebo_model_path):
    tree = ET.parse(world_path)
    root = tree.getroot()

    world = search_single(root, 'world', True, world_path)
    link_dict = dict()

    for child in world:
        if child.tag != 'include':
            continue

        model_prefixes = []
        parse_include(child, world_path, gazebo_model_path, model_prefixes, link_dict, skip_model_ids=skip_model_ids)
    
    return link_dict

def run_parse_world_service():
    rospy.init_node('world_parser')

    try:
        world_path = rospy.get_param("/gazebo_world")
    except:
        rospy.logwarn("/world_name param not set")
        return

    try:
        gazebo_model_path = rospy.get_param("/gazebo_model_path")
    except:
        rospy.logwarn("/gazebo_model_path param not set")
        return

    if not os.path.exists(world_path):
        rospy.logwarn("word path does not exist: " + world_path)

    if not os.path.exists(gazebo_model_path):
        rospy.logwarn("gazebo_model_path does not exist: " + world_path)

    try:
        link_dict = parse_world(world_path, gazebo_model_path)
    except Exception as e:
        rospy.logwarn("failed to parse world: " + str(e))
        raise RuntimeError("Could not parse")

    handler = GazeboModelHandler(link_dict)
    handler_service = rospy.Service('world_parser', GazeboModel, handler.handle_request)

    rospy.spin()

if __name__ == "__main__":
    run_parse_world_service()
