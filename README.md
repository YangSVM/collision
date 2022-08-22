Praticle and Fast Collision detection for autonomous driving.
Welcome to pull request.

用于自动驾驶的快速的碰撞检测代码集合。


code example:
```python
sys.path.append('/home/tiecun/tools_yyb')
from collision_av.collision_detection import is_collision
from collision_av.collision_geometry import Circle, Rectangle

r1 = Rectangle(x=0, y=0, angle_rect, 4, 3)
c1 = Circle(center[0]*d1, center[1]*d1, r=2)
res = is_collision(c1, r1)
print(res)
```

## Method
1. seperating axis therom; 
2. double circle method; 
3. particle model;
4. Future: Minkowski sum

