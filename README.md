# pacman
Pacman Tournament - SegfaultEnthusiasts

Win Condition:
    -Eat all but 2 the dots in enemy territory
    -Or 300 turns go by per agent and have more points


Lose Condition:
    -Lose all but 2 of our dots
    -Have less points by 400 turns per agent
    
    
Offensive Agent:
    Primary Components:
        -Eat all dots in enemy territory
        -Eat all the berries in enemy territory
        -Use the best route to eat all remaining dots and berries
        -Adjust route based off Secondary and Tertiary factors as well
        
    Secondary Components:
        -Avoiding enemy ghosts while eating dots and berries
        -Using berries strategically(i.e. not killing the scared enemy ghost but instead getting as many dots as possible during this time and even finishing off a ghost right before it ends)
        
    Tertiary Components:
        -Using the fact that we know where the enemy ghosts respawn
        -Eating the closest dots to us and as deep into the enemy territory as possible so that we can get the closer ones once we respawn
        -Staying near a berry that we can eat once the enemy ghosts get close to us while in enemy territory
        -Wait out the clock if we have a lead but can't reach the remaining dots


Defensive Agent:

    Primary Components:
        -Patrol our territory
        -Eat any intruder
        -Adjust patrol route based off Secondary and Tertiary factors as well
        
    Secondary Component:
        -Hover the entrances that the enemy ghosts are nearest to
        -Protect the remaining dots (abandon pillaged areas)
        -Avoid letting the pacmen eat the berry (40 turns of being scared lowers our chances of winning)
        
    Tertiary Components:
        -Suicide into an enemy pacman if scared and near (it's faster to respawn and move back than to wait 40 turns)
        -Wait out the clock if we have a lead but can't reach the remaining dots
