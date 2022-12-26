from decentralized_path_auction_python import *

import sys


def test_graph():
    graph = Graph()
    pathway = [graph.insertNode([0, 0, 0, 0])]
    for i in range(1, 10):
        pos = [i, 0, 0, 0]
        node = graph.insertNode(pos)
        assert node
        pathway[-1].edges.append(node)
        node.edges.append(pathway[-1])
        pathway.append(node)

    for node in pathway[1:-1]:
        assert graph.containsNode(node)
        assert len(node.edges) == 2

    for i in range(0, 10):
        pos = [i, 0, 0, 0]
        assert graph.findNode(pos).position == pos

        pos_offset = [i, 10, 0, 0]
        assert graph.findNearestNode(pos_offset, Node.State.DEFAULT).position == pos

        assert graph.removeNode(pos)
        assert not graph.findNode(pos)


def test_path_search():
    graph = Graph()
    nodes = [graph.insertNode([0, 0, 0, 0])]
    for i in range(1, 10):
        pos = [i, 0, 0, 0]
        node = graph.insertNode(pos)
        assert node
        nodes[-1].edges.append(node)
        node.edges.append(nodes[-1])
        nodes.append(node)

    config = PathSearch.Config("A")
    path_search = PathSearch(config)
    path_search.getConfig().price_increment = 2
    assert path_search.getConfig().price_increment == 2
    assert path_search.getConfig().agent_id == config.agent_id
    assert path_search.setDestinations([Destination(nodes[7])]) == PathSearch.Error.SUCCESS
    assert nodes[7] in [dst.node for dst in path_search.getDestinations()]

    path = Path([Visit(nodes[3])])
    assert path_search.iterate(path, 100) == PathSearch.Error.SUCCESS
    assert [visit.node.position[0] for visit in path] == list(range(3, 8))


def test_path_sync():
    graph = Graph()
    nodes = [graph.insertNode([0, 0, 0, 0])]
    for i in range(1, 10):
        pos = [i, 0, 0, 0]
        node = graph.insertNode(pos)
        assert node
        nodes[-1].edges.append(node)
        node.edges.append(nodes[-1])
        nodes.append(node)

    path_a = Path([Visit(n, price=1) for n in nodes])
    path_b = Path([Visit(n, price=2) for n in nodes[6:]])
    path_sync = PathSync()
    assert path_sync.updatePath("A", path_a, path_id=0) == PathSync.Error.SUCCESS
    assert path_sync.updatePath("B", path_b, path_id=0) == PathSync.Error.SUCCESS
    assert (
        path_sync.updateProgress("A", progress_min=2, progress_max=3, path_id=0)
        == PathSync.Error.SUCCESS
    )
    info_a = path_sync.getPaths()["A"]
    assert info_a.progress_min == 2
    assert info_a.progress_max == 3
    info_b = path_sync.getPaths()["B"]
    assert info_b.progress_min == 0
    assert info_b.progress_max == 0
    wait_a = path_sync.checkWaitStatus("A")
    assert wait_a.error == PathSync.Error.SUCCESS
    assert wait_a.blocked_progress == 6
    wait_b = path_sync.checkWaitStatus("B")
    assert wait_b.error == PathSync.Error.SUCCESS
    assert wait_b.blocked_progress == 4

    assert path_sync.removePath("A") == PathSync.Error.SUCCESS
    assert len(path_sync.getPaths()) == 1
    assert path_sync.clearPaths() == PathSync.Error.SUCCESS
    assert len(path_sync.getPaths()) == 0
