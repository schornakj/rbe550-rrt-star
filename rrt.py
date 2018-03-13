import numpy as np
import scipy.ndimage
from PIL import Image, ImageDraw, ImageColor
import random
import math
import time

class Node:
    parent = None
    cost = 0

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def set_parent(self, parent):
        self.parent = parent
        self.cost = math.sqrt(math.pow(self.x - parent.x, 2) + math.pow(self.y - parent.y, 2))


class RRT:
    nodes_start_tree = []
    nodes_goal_tree = []

    def __init__(self, path_to_map, use_rrt, coords_start, coords_end, step_size):
        self.image = Image.open(path_to_map)
        self.map = scipy.ndimage.imread(path_to_map, flatten=True)
        self.root_start = Node(coords_start[0], coords_start[1])
        self.nodes_start_tree.append(self.root_start)

        self.root_goal = Node(coords_end[0], coords_end[1])
        self.nodes_goal_tree.append(self.root_goal)

        self.radius = 50

        self.step_size = step_size

        self.cost_total = 0
        self.path_total = []
        self.success = False

        self.use_rrt = use_rrt

    def dist_euclidean(self, point_a, point_b):
        return math.sqrt(math.pow(point_b[0] - point_a[0], 2) + math.pow(point_b[1] - point_a[1], 2))

    def sample(self):
        x_max = self.map.shape[0]
        y_max = self.map.shape[1]

        goal_roll = random.random()
        if goal_roll <= 0.1:
            return (self.root_goal.x, self.root_goal.y)

        collision_free = False
        while not collision_free:
            point_x = random.random() * x_max
            point_y = random.random() * y_max
            if self.map[int(point_x),int(point_y)] == 0:
                collision_free = True
                return (point_x, point_y)
            # else:
                # print self.map[int(point_x),int(point_y)]
                # print("Cell is occupied!")
        # return (random.random()*x_max, random.random()*y_max)

    def do_extend(self, nodes, current_node, target_position):
        distance_to_target = self.dist_euclidean((current_node.x, current_node.y), (target_position[0], target_position[1]))

        if distance_to_target <= self.step_size:
            new_position_x = target_position[0]
            new_position_y = target_position[1]
        else:
            new_position_x = current_node.x + self.step_size*(target_position[0] - current_node.x)/distance_to_target
            new_position_y = current_node.y + self.step_size*(target_position[1] - current_node.y)/distance_to_target
        closer_node = None
        if self.check_if_obstacle_free((current_node.x, current_node.y), (new_position_x, new_position_y)) and self.node_count < self.num_nodes:
            closer_node = Node(new_position_x, new_position_y)

            # print("Closer node: " + str((closer_node.x, closer_node.y)))
            self.choose_parent(nodes, current_node, closer_node)
            # closer_node.set_parent(current_node)
            # closer_node.cost = closer_node.parent.cost + self.dist_euclidean((closer_node.x, closer_node.y), (current_node.x, current_node.y))

            nodes.append(closer_node)
            self.node_count += 1
            self.rewire(nodes, closer_node)
        return closer_node

    def do_connect(self, nodes, origin_node, target_node):
        current_node = origin_node
        distance_to_target = self.dist_euclidean((current_node.x, current_node.y), (target_node.x, target_node.y))
        distance_from_origin = distance_to_target
        while distance_to_target > self.step_size and self.node_count < self.num_nodes:
            print distance_to_target
            new_position_x = current_node.x + self.step_size*(target_node.x - origin_node.x)/distance_from_origin
            new_position_y = current_node.y + self.step_size*(target_node.y - origin_node.y)/distance_from_origin

            if self.check_if_obstacle_free((current_node.x, current_node.y), (new_position_x, new_position_y)):
                closer_node = Node(new_position_x, new_position_y)
                self.choose_parent(nodes, current_node, closer_node)
                nodes.append(closer_node)
                self.node_count += 1

                current_node = closer_node
            else:
                print "Hit something, unfortunately"
                return False, None, None
            distance_to_target = self.dist_euclidean((current_node.x, current_node.y), (target_node.x, target_node.y))
        if distance_to_target <= self.step_size:
            print("Within range of target")
            return True, current_node, target_node
        else:
            print "Too many nodes"
            return False, None, None

    def steer(self, x, y):
        return NotImplemented

    def get_nearest_node(self, nodes, point):
        distance_min = 10000
        nearest = None
        for node in nodes:
            distance = self.dist_euclidean((node.x, node.y), point)
            if distance < distance_min:
                nearest = node
                distance_min = distance
        return nearest

    def get_near_nodes(self, point, count):
        return NotImplemented

    def check_if_obstacle_free(self, point_a, point_b):
        # x_min = point_a[0]
        # x_max = point_b[0]

        # slope = (point_b[1] - point_a[1])/(x_max-x_min)
        # print(slope)

        # x_range = range(int(math.floor(x_min)), int(math.floor(x_max)))
        # print(x_range)
        # print(x_min, x_max)

        if self.map[int(point_b[1]), int(point_b[0])] > 100:
            print "Free"
            return True
        else:
            print "Obstacle"
            return False

    def find_supercover_squares(self, point_a, point_b):
        points_traversed = []
        points_traversed.append((math.floor(point_a[0]), math.floor(point_a[1])))

        dx = math.floor(point_b[0] - point_a[0])
        dy = math.floor(point_b[1] - point_a[1])
        double_dx = 2*dx
        double_dy = 2*dy

        x_step = 1
        y_step = 1

        x = math.floor(point_a[0])
        y = math.floor(point_a[1])

        if dx < 0:
            x_step = -1
        if dy < 0:
            y_step = -1

        if double_dx >= double_dy:
            errorprev = dx
            error = dx
            for i in np.arange(0, dx):
                x += x_step
                error += double_dy
                if error > double_dx:
                    y += y_step
                    error -= double_dx
                    if error + errorprev < double_dx:
                        points_traversed.append((x, y - y_step))
                    elif error + errorprev > double_dx:
                        points_traversed.append((x-x_step, y))
                    else:
                        points_traversed.append((x, y - y_step))
                        points_traversed.append((x-x_step, y))
                points_traversed.append((x, y))
                errorprev = error
        else:
            errorprev = dy
            error = dy
            for i in np.arange(0, dy):
                y += y_step
                error += double_dx
                if error > double_dy:
                    x += x_step
                    error -= double_dy
                    if error + errorprev < double_dy:
                        points_traversed.append((x - x_step, y))
                    elif error + errorprev > double_dy:
                        points_traversed.append((x, y - y_step))
                    else:
                        points_traversed.append((x, y - y_step))
                        points_traversed.append((x - x_step, y))
                points_traversed.append((x, y))
                errorprev = error
        # assert y is point_b[1] and x is point_b[0]
        return points_traversed

    def rewire(self, nodes, new_node):
        near_nodes = self.get_near_nodes(nodes, (new_node.x, new_node.y))
        for node in near_nodes:
            if self.check_if_obstacle_free((node.x, node.y), (new_node.x, new_node.y))\
                    and new_node.parent is not node \
                    and new_node.cost + self.dist_euclidean((node.x, node.y), (new_node.x, new_node.y)) < node.cost:
                node.parent = new_node
                node.cost = new_node.cost + self.dist_euclidean((node.x, node.y), (new_node.x, new_node.y))

    def choose_parent(self, nodes, nearest_node, new_node):
        nn = nearest_node
        for node in nodes:
            # if this node is within the radius of the new nodw
            # and if the node cost plus distance to the new node is less than for the previous nearest node
            # make it the nearest node
            if self.dist_euclidean((node.x, node.y), (new_node.x, new_node.y)) < self.radius \
                and node.cost + self.dist_euclidean((node.x, node.y), (new_node.x, new_node.y)) \
                    < new_node.cost + self.dist_euclidean((nn.x, nn.y), (new_node.x, new_node.y)):
                nn = node
        # new_node.cost = nn.cost + self.dist_euclidean((nn.x, nn.y), (new_node.x, new_node.y))
        new_node.set_parent(nn)
        new_node.cost = new_node.parent.cost + self.step_size

    def draw_tree(self):
        scale_factor = 20
        image_big = self.image.resize((self.map.shape[0] * scale_factor, self.map.shape[1] * scale_factor), Image.ANTIALIAS).convert('RGB')
        draw = ImageDraw.Draw(image_big)

        for col in range(0,self.map.shape[0]):
            draw.line([(col * scale_factor, 0),
                       (col * scale_factor,self.map.shape[1] * scale_factor)], (0, 0, 0))
        for row in range(0, self.map.shape[1]):
            draw.line([(0, row * scale_factor),
                       (self.map.shape[0] * scale_factor, row * scale_factor)], (0, 0, 0))

        for node in self.nodes_start_tree:
            if node.parent is not None:
                draw.line([(node.x*scale_factor, node.y*scale_factor), (node.parent.x*scale_factor, node.parent.y*scale_factor)], (255,0,255))
                draw.ellipse([(node.x * scale_factor - 2, node.y * scale_factor - 2),
                              (node.x * scale_factor + 2, node.y * scale_factor + 2)], fill=(255,0,255), outline=(255,0,255))
            else:
                draw.ellipse([(node.x * scale_factor - 2, node.y * scale_factor - 2),
                              (node.x * scale_factor + 2, node.y * scale_factor + 2)], fill=(0,255,0), outline=(0,255,0))

        for node in self.nodes_goal_tree:
            if node.parent is not None:
                draw.line([(node.x*scale_factor, node.y*scale_factor), (node.parent.x*scale_factor, node.parent.y*scale_factor)], (0,0,255))
                draw.ellipse([(node.x * scale_factor - 2, node.y * scale_factor - 2),
                              (node.x * scale_factor + 2, node.y * scale_factor + 2)], fill=(0,0,255), outline=(0,0,255))
            else:
                draw.ellipse([(node.x * scale_factor - 2, node.y * scale_factor - 2),
                              (node.x * scale_factor + 2, node.y * scale_factor + 2)], fill=(0,255,255), outline=(0,255,255))

        for i in range(0, len(self.path_total)-1):
            draw.line([(self.path_total[i][0] * scale_factor, self.path_total[i][1] * scale_factor),
                       (self.path_total[i+1][0] * scale_factor, self.path_total[i+1][1] * scale_factor)], (255, 0, 0))


        # draw.line((0, 0) + image_big.size, fill=128)
        # draw.line((0, image_big.size[1], image_big.size[0], 0), fill=128)
        del draw
        image_big.save("test.png")

    def trace_path(self, node):
        path = [(node.x, node.y)]
        next_node = node.parent
        while next_node is not None:
            path.append((next_node.x, next_node.y))
            next_node = next_node.parent
        return path

    def do_RRT(self, num_nodes):
        self.node_count = 0
        self.num_nodes = num_nodes

        start = time.time()

        while self.node_count < self.num_nodes:
            point_random = self.sample()

            nearest = self.get_nearest_node(self.nodes_start_tree, point_random)
            new_node = self.do_extend(self.nodes_start_tree, nearest, point_random)

            # point_random = self.sample()
            if new_node is not None:
                nearest = self.get_nearest_node(self.nodes_goal_tree, (new_node.x, new_node.y))
                trees_linked, joint_node_goal, joint_node_start = self.do_connect(self.nodes_goal_tree, nearest, new_node)
                if trees_linked:
                    path_start_tree = self.trace_path(joint_node_start)
                    path_goal_tree = self.trace_path(joint_node_goal)

                    path_total = path_start_tree
                    path_total.reverse()
                    path_total.extend(path_goal_tree)
                    self.cost_total = joint_node_start.cost + joint_node_goal.cost
                    self.path_total = path_total
                    self.success = True
                    break
        elapsed = time.time() - start
        return elapsed, self.cost_total

    # def do_RRT(self):
    #     V = {x_init}
    #     E = None
    #     i = 0
    #     while i < N:
    #         G = (V, E)
    #         x_rand = sample(i)
    #         i += 1
    #         V, E = extend(G, x_rand)

if __name__ == "__main__":
    random.seed()

    solver = RRT("U_turn.gif", False, (24.5, 24.5), (1.5, 1.5), 1.0)
    time_elapsed, cost = solver.do_RRT(1000)
    print("Time elapsed: " + str(time_elapsed) + " s\nTotal Cost: " + str(cost))
    solver.draw_tree()

    # print("Time elapsed: " + str(elapsed) + " s")
    # print(solver.find_supercover_squares((0.0, 0.5), (1, 1)))
    # print("Total Cost: " + str(solver.cost_total))
