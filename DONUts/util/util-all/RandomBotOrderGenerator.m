%%Generates an array with the robots listed in a random order.
function robotsRandom = RandomBotOrderGenerator(numBots)
robots = 1:numBots;
robotsRandom = robots(randperm(length(robots)));