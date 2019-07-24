extends Node2D

var visual_grid_w
var visual_grid_h
var visual_grid = {}
var obstacles = {}

var current_astar
var other_astar

const USE_DIAGONALS = true

func _test_normal_simple():
	
	var new_astar = AStarGrid2D.new()
	new_astar.resize(1000, 1000)
	
	new_astar.connect_points(Vector2(0, 0), Vector2(0, 1), 1)
	new_astar.connect_points(Vector2(0, 1), Vector2(0, 2), 1)
	new_astar.connect_points(Vector2(0, 2), Vector2(1, 2), 1)
	
	var path = new_astar.get_grid_path(Vector2(0, 0), Vector2(1, 2))
	

func _test_normal_astar(grid_width, grid_height):
	
	var normal_astar = AStar2D.new()
	
	var i = 0
	for x in grid_width:
		for y in grid_height:
			normal_astar.add_point(i, Vector2(x, y))
			i += 1
	
	for i in grid_width * grid_height:
		if i < grid_height * grid_width - 1:
			normal_astar.connect_points(i, i + 1)
	
	var start_normal_usec = OS.get_ticks_usec()
	var normal_path = normal_astar.get_point_path(0, grid_width * grid_height - 1)
	var took_normal_usec = OS.get_ticks_usec() - start_normal_usec
	print("normal astar took: %d usec" % took_normal_usec)
	var norm_elem_per_usec = (grid_width * grid_height) / float(took_normal_usec)
	print("elements per usec: %f" % norm_elem_per_usec)
	# print(normal_path)

func _test_our_astar(grid_width, grid_height):
	
	var our_astar = AStarGrid2D.new()
	our_astar.resize(grid_width, grid_height)
	
	for x in grid_width:
		for y in grid_height:
			if x != grid_width - 1:
				our_astar.connect_points(Vector2(x, y), Vector2(x + 1, y), 1)
			else:
				our_astar.connect_points(Vector2(x, y), Vector2(x, y + 1), 1)
	
	var start_our_usec = OS.get_ticks_usec()
	var bigge_path = our_astar.get_grid_path(Vector2(0, 0), Vector2(grid_width - 1, grid_height - 1))
	var took_our_usec = OS.get_ticks_usec() - start_our_usec
	print("our astar took: %d usec" % took_our_usec)
	var our_elem_per_usec = (grid_width * grid_height) / float(took_our_usec)
	print("elements per usec: %f" % our_elem_per_usec)
	# print(bigge_path)
	
	return bigge_path

func position_to_index(width, height, pos):
	var x = pos.x
	var y = pos.y
	if x < 0 or x >= width or y < 0 or y >= height: return -1
	var i = (x * height) + y
	return int(i)

func index_to_position(width, height, i):
	var x = i % height
	var y = i / height
	return Vector2(x, y)

func _test_default_fully_connected_astar(grid_width, grid_height):
	
	var astar = AStar2D.new()
	
	var i = 0
	for x in grid_width:
		for y in grid_height:
			astar.add_point(i, Vector2(x, y))
			i += 1
	
	i = 0
	var num_conns = 0
	for x in grid_width:
		for y in grid_height:
			
			var cur_pos = Vector2(x, y)
			var middle = position_to_index(grid_width, grid_height, cur_pos)
			if middle in obstacles: continue
			
			if x > 0:
				var left_pos = cur_pos + Vector2(-1, 0)
				var left = position_to_index(grid_width, grid_height, left_pos)
				if left_pos in obstacles: continue
				astar.connect_points(middle, left)
				num_conns += 1
			
			if y < grid_height - 1:
				var bottom_pos = cur_pos + Vector2(0, 1)
				var bottom = position_to_index(grid_width, grid_height, bottom_pos)
				if bottom_pos in obstacles: continue
				astar.connect_points(middle, bottom)
				num_conns += 1
			
			if x < grid_width - 1:
				var right_pos = cur_pos + Vector2(1, 0)
				var right = position_to_index(grid_width, grid_height, right_pos)
				if right_pos in obstacles: continue
				astar.connect_points(middle, right)
				num_conns += 1
			
			if y > 0:
				var top_pos = cur_pos + Vector2(0, -1)
				var top = position_to_index(grid_width, grid_height, top_pos)
				if top_pos in obstacles: continue
				astar.connect_points(middle, top)
				num_conns += 1
			
			if x > 0 && y > 0 and USE_DIAGONALS:
				var top_left_pos = cur_pos + Vector2(-1, -1)
				var top_left = position_to_index(grid_width, grid_height, top_left_pos)
				if top_left_pos in obstacles: continue
				astar.connect_points(middle, top_left)
				num_conns += 1
			
			if x < grid_width - 1 && y > 0 and USE_DIAGONALS:
				var top_right_pos = cur_pos + Vector2(1, -1)
				var top_right = position_to_index(grid_width, grid_height, top_right_pos)
				if top_right in obstacles: continue
				astar.connect_points(middle, top_right)
				num_conns += 1
			
			if x > 0 && y < grid_height - 1 and USE_DIAGONALS:
				var bottom_left_pos = cur_pos + Vector2(-1, 1)
				var bottom_left = position_to_index(grid_width, grid_height, bottom_left_pos)
				if bottom_left_pos in obstacles: continue
				astar.connect_points(middle, bottom_left)
				num_conns += 1
			
			if x < grid_width - 1 && y < grid_height - 1 and USE_DIAGONALS:
				var bottom_right_pos =  cur_pos + Vector2(1, 1)
				var bottom_right = position_to_index(grid_width, grid_height, bottom_right_pos)
				if bottom_right_pos in obstacles: continue
				astar.connect_points(middle, bottom_right)
				num_conns += 1
			
			i += 1
	
	var start_normal_usec = OS.get_ticks_usec()
	var normal_path = astar.get_point_path(0, grid_width * grid_height - 1)
	var took_normal_usec = OS.get_ticks_usec() - start_normal_usec
	print("normal astar took: %d usec" % took_normal_usec)
	var norm_elem_per_usec = (grid_width * grid_height) / float(took_normal_usec)
	print("elements per usec: %f" % norm_elem_per_usec)
	print("number of steps: %d" % normal_path.size())
	# print(normal_path)
	
	return [astar, normal_path]
	

func _test_our_fully_connected_astar(grid_width, grid_height):
	
	var our_astar = AStarGrid2D.new()
	our_astar.resize(grid_width, grid_height)
	
	for x in grid_width:
		for y in grid_height:
			our_astar.connect_to_neighbours(Vector2(x, y), 1, USE_DIAGONALS)
	
	for x in grid_width:
		for y in grid_height:
			var cur_pos = Vector2(x, y)
			if cur_pos in obstacles:
				our_astar.disconnect_from_neighbours(cur_pos)
	
	var start_our_usec = OS.get_ticks_usec()
	var bigge_path = our_astar.get_grid_path(Vector2(0, 0), Vector2(grid_width - 1, grid_height - 1))
	var took_our_usec = OS.get_ticks_usec() - start_our_usec
	print("our astar took: %d usec" % took_our_usec)
	var our_elem_per_usec = (grid_width * grid_height) / float(took_our_usec)
	print("elements per usec: %f" % our_elem_per_usec)
	print("number of steps: %d" % bigge_path.size())
	# print(bigge_path)
	
	return [our_astar, bigge_path]

func _generate_random_obstacles(w, h, max_obstacles):
	
	var count = int(rand_range(0, max_obstacles))
	
	for n in count:
		var x = int(rand_range(0, w))
		var y = int(rand_range(0, h))
		var size = int(rand_range(0, 8))
		var l_x = x - (size / 2)
		var l_y = y - (size / 2)
		var center = Vector2(x, y)
		for t_x in size:
			for t_y in size:
				var c = Vector2(l_x + t_x, l_y + t_y)
				if c.distance_to(center) <= size / 2:
					obstacles[c] = true

func _draw():
	
	var vp_size = get_viewport_rect().size
	var g_w = vp_size.x / float(visual_grid_w)
	var g_h = vp_size.y / float(visual_grid_h)
	var t_w = g_w
	var t_h = g_h
	
	for o in obstacles:
		draw_rect(Rect2(o.x * g_w, o.y * g_h, t_w, t_h), Color.pink)
	
	for e in visual_grid:
		draw_rect(Rect2(e.x * g_w, e.y * g_h, t_w, t_h), visual_grid[e])

func _draw_grid(path, color, clear=true):
	if clear: visual_grid.clear()
	for e in path:
		visual_grid[e] = color

func _mark_inaccessible():
	for x in visual_grid_w:
		for y in visual_grid_h:
			var target = Vector2(x, y)
			var p
			if current_astar.has_method("get_grid_path"):
				p = current_astar.get_grid_path(Vector2(0, 0), target)
			else:
				p = current_astar.get_point_path(0, position_to_index(visual_grid_w, visual_grid_h, target))
			if p.size() == 0 and target.distance_to(Vector2(0, 0)) > 1:
				visual_grid[target] = Color.red
			else:
				visual_grid[target] = Color.green

func _ready():
	
	var grid_width = 128
	var grid_height = 128
	
	_generate_random_obstacles(grid_width, grid_height, 512)
	
	# _test_normal_simple()
	# _test_normal_astar(grid_width, grid_height)
	# var our_path = _test_our_astar(grid_width, grid_height)
	
	var normal = _test_default_fully_connected_astar(grid_width, grid_height)
	var normal_astar = normal[0]
	var normal_path = normal[1]
	
	var our = _test_our_fully_connected_astar(grid_width, grid_height)
	var our_astar = our[0]
	var our_path = our[1]
	
	visual_grid_w = grid_width
	visual_grid_h = grid_height
	current_astar = our_astar
	other_astar = normal_astar
	_draw_grid(our_path, Color.red)
	_draw_grid(normal_path, Color.green, false)
	
	# _mark_inaccessible()

func _bench_astar(which, astar, from_pos, to_pos):
	var new_path
	var start_usec
	if astar.has_method("get_grid_path"):
		start_usec = OS.get_ticks_usec()
		new_path = astar.get_grid_path(Vector2(0, 0), to_pos)
	else:
		var from_id = astar.get_closest_point(from_pos)
		var target_id = astar.get_closest_point(to_pos)
		start_usec = OS.get_ticks_usec()
		new_path = astar.get_point_path(from_id, target_id)
	var took_usec = OS.get_ticks_usec() - start_usec
	print("%s astar took: %d usec" % [which, took_usec])
	var elem_per_usec = (visual_grid_w * visual_grid_w) / float(took_usec)
	print("elements per usec: %f" % elem_per_usec)
	print("number of steps: %d \n" % new_path.size())
	return new_path

func _input(event):
	if event is InputEventMouseButton:
		if event.pressed and event.button_index == BUTTON_LEFT:
			var vp_size = get_viewport_rect().size
			var click_pos = event.position
			var click_grid_x = click_pos.x / (vp_size.x / float(visual_grid_w))
			var click_grid_y = click_pos.y / (vp_size.y / float(visual_grid_h))
			var click_grid_pos = Vector2(click_grid_x, click_grid_y)
			
			var first_path = _bench_astar("our", current_astar, Vector2(0, 0), click_grid_pos)
			var second_path = _bench_astar("default", other_astar, Vector2(0, 0), click_grid_pos)
			_draw_grid(first_path, Color.green)
			_draw_grid(second_path, Color.red, false)
			update()
