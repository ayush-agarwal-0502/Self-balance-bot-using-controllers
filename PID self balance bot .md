# Self Balance Bot using PID controller 
## Introduction 
This project was given to us as a problem statment in the final week of controls and dynamics camp , robotics summer camp IIT BHU Varanasi . The link for the PS page is here .
https://github.com/Robotics-Club-IIT-BHU/CnD-SummerCamp21/tree/master/Week%203/The%20Final%20Challenge
The pybullet code files for this bot are avaiaible on this repository .
## PID controller working 
This page explains it much better than I do , so I'll paste the link here , so that I can focus on the project demostration more . 
https://github.com/Robotics-Club-IIT-BHU/CnD-SummerCamp21/tree/master/Week%201/SubPart%202
## Project demonstration 
The bot balances itself as the PID controller inside it sends correction signals to the bot . 
The bot moves forward when torque is applied in such a way that the wheels make the bot move forward . The PID controller keeps sending corrective signals so the bot can still maintain it's balance properly . That's how the bot moves forward or backward .
<video width="320" height="240" >
  <source src="https://github.com/ayush-agarwal-0502/Self-balance-bot-using-controllers/blob/main/PID%20bot%20videos/bot%20moving%20forward.mp4" type="mp4">
</video>
To move left or right , the wheels rotate in the opposite direction , this causes turning effect in the bot . This concept of turning of robot due to difference in wheel velocities is called as differential drive . 

The bot can also perform a front flip as well as a back flip, as visible in the videos . This is possible , since in pybullet environment , we do not have a limit to the amount of torque the motors can apply , hence the motors apply so much torque that the bot ends up doing a flip . After landing , the PID controller keeps sending the correction signals hence the bot ends up standing properly again . 

