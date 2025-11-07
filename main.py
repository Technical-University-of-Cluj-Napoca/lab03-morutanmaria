from utils import *
from grid import Grid
from collections import defaultdict 
from searching_algorithms import *


if __name__ == "__main__":
    # setting up how big will be the display window
    WIN = pygame.display.set_mode((WIDTH, HEIGHT))

    # set a caption for the window
    pygame.display.set_caption("Path Visualizing Algorithm")

    ROWS = 50  # number of rows
    COLS = 50  # number of columns
    grid = Grid(WIN, ROWS, COLS, WIDTH, HEIGHT)

    start = None
    end = None

    # flags for running the main loop
    run = True
    started = False
    # map for gbfs
    region_map = {
    (0, 0): 1, (0, 1): 1, (0, 2): 1, (0, 3): 1, (0, 4): 1,
    (1, 0): 1, (1, 1): 1, (1, 2): 1, (1, 3): 1, (1, 4): 1,
    (2, 0): 1, (2, 1): 1, (2, 2): 1, (2, 3): 1, (2, 4): 1,

    (3, 0): 3, (3, 1): 3, (3, 2): 3, (3, 3): 3,
    (4, 0): 3, (4, 1): 3, (4, 2): 3, (4, 3): 3,

    (5, 0): 5, (5, 1): 5, (5, 2): 5, (5, 3): 5,
    (6, 0): 5, (6, 1): 5, (6, 2): 5, (6, 3): 5,

    (7, 0): 8, (7, 1): 8, (7, 2): 8,
    (8, 0): 8, (8, 1): 8, (8, 2): 8,

    (3, 5): 12, (4, 5): 12, (5, 5): 12,
    (3, 6): 12, (4, 6): 12, (5, 6): 12,

    (2, 7): 20, (3, 7): 20, (4, 7): 20,
    (2, 8): 20, (3, 8): 20, (4, 8): 20,

    (6, 7): 999, (6, 8): 999,
    (7, 7): 999, (7, 8): 999,
}

    while run:
        grid.draw()  # draw the grid and its spots
        for event in pygame.event.get():
            # verify what events happened
            if event.type == pygame.QUIT:
                run = False

            if started:
                # do not allow any other interaction if the algorithm has started
                continue  # ignore other events if algorithm started

            if pygame.mouse.get_pressed()[0]:  # LEFT CLICK
                pos = pygame.mouse.get_pos()
                row, col = grid.get_clicked_pos(pos)

                if row >= ROWS or row < 0 or col >= COLS or col < 0:
                    continue  # ignore clicks outside the grid

                spot = grid.grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()
                elif not end and spot != start:
                    end = spot
                    end.make_end()
                elif spot != end and spot != start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # RIGHT CLICK
                pos = pygame.mouse.get_pos()
                row, col = grid.get_clicked_pos(pos)
                spot = grid.grid[row][col]
                spot.reset()

                if spot == start:
                    start = None
                elif spot == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not started:
                    # run the algorithm
                    for row in grid.grid:
                        for spot in row:
                            spot.update_neighbors(grid.grid)
                    started = False
                    # here you can call the algorithms
                    #bfs key = b
                if event.key == pygame.K_b and not started:
                    for row in grid.grid:
                        for spot in row:
                            spot.update_neighbors(grid.grid)
                    bfs(lambda: grid.draw(), grid, start, end)
                    #dfs key = d
                if event.key == pygame.K_d and not started:
                    for row in grid.grid:
                        for spot in row:
                            spot.update_neighbors(grid.grid)
                    dfs(lambda: grid.draw(), grid, start, end)  
                    #astar key = a
                if event.key == pygame.K_a and not started:
                    for row in grid.grid:
                        for spot in row:
                            spot.update_neighbors(grid.grid)
                    astar(lambda: grid.draw(), grid, start, end)
                    # ... and the others?
                    #dls key = l
                if event.key == pygame.K_l and not started:
                    for row in grid.grid:
                        for spot in row:
                            spot.update_neighbors(grid.grid)
                    dls(lambda: grid.draw(), grid, start, end, 50)
                    #ucs key = u
                if event.key == pygame.K_u and not started:
                    for row in grid.grid:
                        for spot in row:
                            spot.update_neighbors(grid.grid)
                    ucs(lambda: grid.draw(), grid, start, end) 
                    #greedy best first search key = g
                if event.key == pygame.K_g and not started:
                    for row in grid.grid:
                        for spot in row:
                            spot.update_neighbors(grid.grid)
                    gbfs(lambda: grid.draw(), grid, start, end, region_map) 

                if event.key == pygame.K_c:
                    print("Clearing the grid...")
                    start = None
                    end = None
                    grid.reset()
    pygame.quit()
