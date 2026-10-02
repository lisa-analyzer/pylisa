def f(node, String):
    return node.create_publisher(String, '/robot/' 'chatter', 10)
