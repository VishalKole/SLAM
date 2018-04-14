import matplotlib.image as img, math, heapq as hq

"""
vishal kole
"""

class Heuristics:
    def getHeuristicCost(self, current, destination):
        return math.sqrt(math.pow((current[0] - destination[0]), 2) +
                         math.pow((current[1] - destination[1]), 2))


class Astar:
    def __init__(self, fmap, heuristics):
        self.map = fmap
        self.heuristics = heuristics
        self.heap = []
        self.trace = []

    def childIn(self, child, child_list):
        for tuples in child_list:
            if tuples[1][0] == child[0] and tuples[1][1] == child[1]:
                return True
        return False

    def searchPath(self, start, destination):
        hq.heappush(self.heap, (0, start))
        while self.heap:
            current_element = hq.heappop(self.heap)
            self.trace.append(current_element)
            if current_element[1][0] == destination[0] and current_element[1][1] == destination[1]:
                ret = []
                for items in self.trace:
                    ret.append(items[1])
                self.trace.clear()
                self.heap.clear()
                return ret
            else:
                childrens = self.map.getChildren(current_element[1])
                for childs in childrens:
                    if not self.childIn(childs, self.heap) and not self.childIn(childs, self.trace):
                        hq.heappush(self.heap,
                                    # (current_element[0] + 2.0 +
                                    (self.heuristics.getHeuristicCost(childs, destination), childs))


class CustomMap:
    def __init__(self, fmap):
        self.map = fmap

    def getChildren(self, point):
        childrens = []
        for i in [-1, +1, 0]:
            for j in [-1, +1, 0]:
                if self.map[point[0] + i][point[1] + j] == 1.0:
                    childrens.append([point[0] + i, point[1] + j])
        childrens.pop()
        return childrens


def main():
    cmap = CustomMap(img.imread("project.png"))
    heuristics = Heuristics()
    search = Astar(cmap, heuristics)
    start = [270, 1130]
    destination = [272, 1132]
    print(search.searchPath(start, destination))


if __name__ == '__main__':
    main()
