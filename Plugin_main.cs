



//Alexandre de Montleau - 20/12/22
//Semester project @Biorob: Improving Roombot's docking procedure


using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Threading;

using UnityEngine;
using UnityEngine.Collections;

namespace Plugins
{

    /// <summary>
    /// Template class for a new plugin.
    /// This example plugin does :
    ///     - read the parameters in the initialization to decide which module to interact with 
    ///     (if no parameters given, use the first module in the list)
    ///     - reset the position of the three motors to 0 at startup
    ///     - automatically opens and closes ACM0
    ///     - manually rotates Motor1 with left and right arrow keys and joystick axis 1
    ///     - manually rotates Motor0 with joystick axis 2
    ///     - manually rotates Motor2 with joystick axis 3
    ///     - manually opens/closes ACM1 with joystick button 0
    ///     - stop the execution of the plugin with F12 key
    /// </summary>
    public class Template : Plugin
    {
        //RBStatus 
        private static RBStatus rb;
        // RB module to interact with
        private static int _RB_id = -1;
        // ACM to interact with
        private static int _ACM = -1;
        //motor order to change
        private static int[] _motor_order;
        //task to do in update 
        private static int _task = -1;
        //task to do after wait 
        private static int _task_after_wait = -1;
        //time tracking 
        private static float mytime = Time.time;
        //list of commands
        private static List<string> listOfCommands = new List<string>();

        //delta
        private static List<float[]> _delta = new List<float[]>();

        //desired theta command
        private static float[] _theta_desired = new float[] { 0.0f, 0.0f, 0.0f };

        //desired theta command
        private static List<float[]> _theta_visited = new List<float[]>();

        //baseline theta command
        private static float[] _theta_baseline = new float[] { 0.0f, 0.0f, 0.0f };

        //desired theta command
        private static float[] _angle_limits = new float[] { 0.0f, 0.0f, 0.0f };
        //desired theta brs
        private static float[] _theta_brs = new float[] { 0, 0, 0 };


        //last command 
        private static string _last_command = null;
        //wait counter 
        private static int _counter = 0;
        //random numbers
        private static System.Random rnd = new System.Random();
        //first trial 
        private static bool first_trial = true;
        //try cases 
        private static int try_cases = 0;
        //objective function 
        private static List<double> _f = new List<double>();
        private static double f_min = 0;
        //most probably blocked variable 
        private static int blocked_counter = 0;


        //BRS
        private static int n;
        public const int N = 4;
        public const int max_randm = 4;
        public const double alpha = 1;
        private static int[,] increment_table;
        private static int[,] std_multiplier = new int[7, 3] { { 2, 1, 1 }, { 1, 2, 1 }, { 1, 1, 2 }, { 2, 2, 1 }, { 1, 2, 2 }, { 2, 1, 2 }, { 2, 2, 2 } };
        private static int[] STD = new int[] { 1, 1, 1 };
        private static bool ignore_best = false;
        private static int align_trial = 0;
        private static int track_multiplier;
        private static double pitch = 0.0;
        private static double pitch0 = 0.0;
        private static double pitch1 = 0.0;


        public const int check_other_acm = 0;
        public const int calculate_pitch = 1;
        public const int increment_acm = 2;
        public const int wait = 3;
        public const int process_listOfCommands = 4;
        public const int check_acm = 5;
        public const int ask_acm = 6;
        public const int quit = 7;
        public const int start_align = 8;
        public const int fill_list = 9;
        public const int done = 10;


        public static float normal_hall = 4908;
        public static float normal_IR = 8162;

        public static float cost_norm_hall = 1;
        public static float cost_norm_ir = 1;



        /// <summary>
        /// saturation function for angles 
        /// </summary>
        /// <returns> saturated value if value is outside the pos limits. </returns>
        private static float limit_angle(float value, float max, float min)
        {
            if (value >= min && value <= max)
                return value;
            else if (value > max)
                return max;
            else
                return min;

        }



        // ACM to open/close

        /// <summary>
        /// Called only once at plugin startup
        /// </summary>
        /// <param name="parameters">Parameters for initializion, in an array of strings</param>
        /// <returns>The list of commands to execute in the GUI and/or send to the Roombots modules</returns>
        public static List<string> initialization(string[] parameters = null)
        {
            //// TODO PROCESS parameters ////
            switch (parameters.Count())
            {
                case 2:
                    _RB_id = Int32.Parse(parameters[0]);
                    _ACM = Int32.Parse(parameters[1]);
                    if (_ACM == 0)
                    {
                        _theta_baseline = new float[] { -120.0f, 180.0f, 120.0f };
                        _theta_desired = new float[] { -120.0f, 180.0f, 120.0f };
                    }

                    else
                    {
                        _theta_desired = new float[] { 120.0f, 180.0f, -120.0f };
                        _theta_baseline = new float[] { 120.0f, 180.0f, -120.0f };
                    }
                    break;
                case 5:
                    _RB_id = Int32.Parse(parameters[0]);
                    _ACM = Int32.Parse(parameters[1]);
                    _theta_desired[0] = float.Parse(parameters[2]);
                    _theta_desired[1] = float.Parse(parameters[3]);
                    _theta_desired[2] = float.Parse(parameters[4]);

                    _theta_baseline[0] = float.Parse(parameters[2]);
                    _theta_baseline[1] = float.Parse(parameters[3]);
                    _theta_baseline[2] = float.Parse(parameters[4]);
                    break;
                default:
                    _RB_id = Plugin._rbList.Keys.First();
                    _ACM = 0;
                    break;
            }

            if (_ACM == 0)
                _motor_order = new int[] { 2, 1, 0 };
            else
                _motor_order = new int[] { 0, 1, 2 };

            _task = ask_acm;
            first_trial = true;
            align_trial = 0;
            _f = new List<double>();
            blocked_counter = 0;
            _counter = 0;
            try_cases = 0;
            track_multiplier = 0;
            increment_table = new int[26, 3] {

            {1, 0, 0}, {-1, 0, 0},
            {1, 0, 1}, {-1, 0, 1},  {0, 0, 1},
            {1, 0,-1}, {-1, 0,-1},  {0, 0,-1},
            {1, 1,-1}, {-1, 1,-1},  {0, 1,-1},
            {1, 1, 0}, {-1, 1, 0},  {0, 1, 0},
            {1, 1, 1}, {-1, 1, 1},  {0, 1, 1},
            {1,-1,1},  {-1, -1,1},  {0, -1,1},
            {1,-1,-1}, {-1, -1,-1}, {0, -1,-1},
            {1, -1,0}, {-1, -1,0},  {0, -1,0}
        };

            n = 0;
            for (int i = 0; i < 3; i++)
            {
                _theta_brs[i] = _theta_desired[i];
                _angle_limits[i] = _theta_desired[i];
            }
            _f.Clear();
            _delta.Clear();
            _theta_visited.Clear();
            List<string> commands = new List<string>();
            RBCommand command = null;

            command = new RBCommand("PRINT ID " + _RB_id + " ACM : " + _ACM + " theta: " + _theta_desired[0].ToString() + _theta_desired[1].ToString() + _theta_desired[2].ToString());
            commands.Add(command.toString);

            return commands;

        }


        /// <summary>
        /// Called at a fixed frequency (10 times per second by default)
        /// </summary>
        /// <returns>The list of commands to execute in the GUI and/or send to the Roombots modules</returns>
        public static List<string> update()
        {
            List<string> commands = new List<string>();
            RBCommand command = null;
            rb = Plugin._rbList[_RB_id];
            //command = new RBCommand("PRINT time: " + Time.time + " " + rb.toString());
            //commands.Add(command.toString);
            // command = new RBCommand("PRINT in update");
            // commands.Add(command.toString);
            switch (_task)
            {
                case ask_acm:
                    command = new RBCommand("PRINT in ASK_ACM");
                    commands.Add(command.toString);
                    command = new RBCommand(Config.RB_COMMAND_ASK, 'P', _ACM, _RB_id, "x", ""); //check the acm in task 
                    commands.Add(command.toString);
                    mytime = Time.time;



                    _task = wait;
                    _task_after_wait = check_acm;
                    break;


                case check_acm:
                    if (Time.time - mytime > 0.5) //wait for the msg to be received. 
                    {
                        cost_norm_hall = (rb.acmHall[_ACM][0] + rb.acmHall[_ACM][1] + rb.acmHall[_ACM][2] + rb.acmHall[_ACM][3]) / normal_hall;
                        cost_norm_ir = ((rb.acmIR[_ACM][0] + rb.acmIR[_ACM][1]) / normal_IR);

                        _f.Add((double)(cost_norm_hall + cost_norm_ir));//normalized sum of hall effect sensors and IR sensors
                        

                        command = new RBCommand("PRINT cost norm hall: " + cost_norm_hall.ToString());
                        commands.Add(command.toString);
                        command = new RBCommand("PRINT cost norm ir: " + cost_norm_ir.ToString());
                        commands.Add(command.toString);


                        float[] this_theta = new float[] { 0, 0, 0 };
                        for (int i = 0; i < 3; i++)
                            this_theta[i] = _theta_desired[i];
                        command = new RBCommand("PRINT theta in check_acm: " + this_theta[0].ToString() + " " + this_theta[1].ToString() + " " + this_theta[2].ToString());
                        commands.Add(command.toString);
                        _theta_visited.Add(this_theta);
                        mytime = Time.time;
                        //bool contains=_theta_visited.Contains(this_theta);
                        //command = new RBCommand("PRINT contains: " + contains.ToString());
                        //commands.Add(command.toString);

                        command = new RBCommand("PRINT IN CHECK_ACM");
                        commands.Add(command.toString);


                        //if all acm Hall sensors are OK exit... 
                        if (rb.acmHall[_ACM][0] < Config.RB_ACM_THRESH_HALL && rb.acmHall[_ACM][1] < Config.RB_ACM_THRESH_HALL && rb.acmHall[_ACM][2] < Config.RB_ACM_THRESH_HALL && rb.acmHall[_ACM][3] < Config.RB_ACM_THRESH_HALL)
                        {
                            _task = increment_acm;
                            _task_after_wait = quit;
                            command = new RBCommand("PRINT _f: " + (rb.acmHall[_ACM][0] + rb.acmHall[_ACM][1] + rb.acmHall[_ACM][2] + rb.acmHall[_ACM][3]) / 5000.0);
                            commands.Add(command.toString);
                        }
                        else
                            _task = start_align;
                        if (align_trial >= 100)
                            _task = quit;
                    }
                    break;

                case check_other_acm:
                    command = new RBCommand("PRINT command ask for acm data");
                    commands.Add(command.toString);
                    command = new RBCommand(Config.RB_COMMAND_ASK, 'P', 1 - _ACM, _RB_id, "x", ""); //check the other acm attached. 
                    commands.Add(command.toString);
                    mytime = Time.time;
                    _task = calculate_pitch;
                    break;

                case calculate_pitch:

                    if (Time.time - mytime > 0.5) // wait for 0.5 seconds for the acm to be updated... 
                    {
                        mytime = Time.time;
                        // Get the status of the Roombots module in the list
                        //double pitch;
                        //pitch = 180 * Math.Atan2(-(rb.acmAcc[1 - _ACM][2] - 2048), Math.Sqrt((double)(Math.Pow(rb.acmAcc[1 - _ACM][0] - 2048, 2) + Math.Pow(rb.acmAcc[1 - _ACM][1] - 2048, 2)))) / Math.PI;
                        //command = new RBCommand("PRINT calculated pitch is: " + pitch.ToString());
                        //commands.Add(command.toString);
                        // command = new RBCommand("PRINT accel are: " + rb.acmAcc[1 - _ACM][0].ToString() + "  " + rb.acmAcc[1 - _ACM][1].ToString() + "  " + rb.acmAcc[1 - _ACM][2].ToString());
                        //  commands.Add(command.toString);

                        //given this pitch calculate the "new" angles to reach the destination. 
                        _theta_desired[0] = _theta_desired[0] - 2;
                        _theta_desired[1] = _theta_desired[1] + 1;
                        _theta_desired[2] = _theta_desired[2] - 2;

                        _task = fill_list;
                    }
                    break;

                case fill_list:
                    command = new RBCommand("PRINT fill_list");
                    commands.Add(command.toString);
                    mytime = Time.time;

                    float[] _theta_now;
                    _theta_now = rb.axesAngle;

                    bool first = true;
                    bool _on_wall = false;
                    bool _0_goback = false;
                    foreach (int i in _motor_order)
                    {
                        if (Math.Abs(_theta_now[i] - _theta_desired[i]) == 0)
                        {
                            command = new RBCommand("PRINT do nothing"); //print 
                            commands.Add(command.toString);
                        }
                        else if (Math.Abs(_theta_now[i] - _theta_desired[i]) > 0 && Math.Abs(_theta_now[i] - _theta_desired[i]) < 10) //if the difference is larger than 3 degrees it will go to new value. 
                        {
                            command = new RBCommand(Config.RB_COMMAND_SET, 'M', i, _RB_id, "pa", ((int)(_theta_desired[i] * 10)).ToString()); //set new
                            listOfCommands.Add(command.toString);

                            if (first)
                                _on_wall = true;
                        }
                        else //else do back and then go back to the new value. 
                        {
                            //command = new RBCommand(Config.RB_COMMAND_SET, 'X', _ACM, _RB_id, "pos", "0");
                            //listOfCommands.Add(command.toString);
                            if (_on_wall)
                            {
                                command = new RBCommand(Config.RB_COMMAND_SET, 'M', _motor_order[0], _RB_id, "pa", (Math.Sign(_theta_now[_motor_order[0]]) * (int)(Math.Abs(_theta_now[_motor_order[0]]) /*- 20*/) * 10).ToString());//go back a bit 
                                listOfCommands.Add(command.toString);
                                _0_goback = true;
                            }
                            command = new RBCommand(Config.RB_COMMAND_SET, 'M', i, _RB_id, "pa", (Math.Sign(_theta_now[i]) * ((int)(Math.Abs(_theta_now[i]) /*- 20*/) * 10)).ToString());//go back a bit 
                            listOfCommands.Add(command.toString);

                            if (!first)
                            {
                                command = new RBCommand(Config.RB_COMMAND_SET, 'M', i, _RB_id, "pa", ((int)(_theta_desired[i] * 10)).ToString());//go to desired angle again
                                listOfCommands.Add(command.toString);
                            }
                            else
                                _0_goback = true;

                        }
                        first = false;
                    }
                    if (_0_goback)
                    {
                        //command = new RBCommand(Config.RB_COMMAND_SET, 'X', _ACM, _RB_id, "pos", "3");//in fact should be 3 if we want to have the acm out 
                        //listOfCommands.Add(command.toString);
                        command = new RBCommand(Config.RB_COMMAND_SET, 'M', _motor_order[0], _RB_id, "pa", ((int)(_theta_desired[_motor_order[0]] * 10)).ToString());//go to desired angle again
                        listOfCommands.Add(command.toString);
                    }

                    _task = wait;
                    _task_after_wait = process_listOfCommands;
                    if (listOfCommands.Any())
                    {
                        _last_command = listOfCommands[0];
                        commands.Add(listOfCommands[0]);
                        listOfCommands.RemoveAt(0);
                    }

                    command = new RBCommand("PRINT end fill_list");
                    commands.Add(command.toString);
                    break;

                case increment_acm:  //increment the acm by 10. 
                    mytime = Time.time;
                    command = new RBCommand(Config.RB_COMMAND_SET, 'X', _ACM, _RB_id, "pos", (Saturation(rb.acmPosition[_ACM] + 5)).ToString());
                    commands.Add(command.toString);
                    _task = _task_after_wait;
                    break;

                case wait: //wait for 0.5 sec and check if all angles are reached in a margin of 2 degrees. 
                           //command = new RBCommand("PRINT wait: ");
                           //commands.Add(command.toString);

                    if (Time.time - mytime > 0.5)
                    {
                        _counter++;
                        mytime = Time.time;
                        if (Math.Abs(rb.axesAngle[0] - rb.axesDesiredAngle[0]) < 1 && Math.Abs(rb.axesAngle[1] - rb.axesDesiredAngle[1]) < 1 && Math.Abs(rb.axesAngle[2] - rb.axesDesiredAngle[2]) < 1) //if 
                        {
                            _task = _task_after_wait;
                            _counter = 0;
                        }
                    }
                    if (_counter > 5) //after 2 sec 
                    {
                        int motor_nb = 0;
                        blocked_counter++;
                        _counter = 0;
                        if (blocked_counter > 2) //after 4 s: most probably blocked so ignore this command: set the desired angle to my angles now. 
                        {
                            blocked_counter = 0;
                            motor_nb = (_last_command[2]) - '0';
                            commands.Add(new RBCommand("PRINT motor is : " + motor_nb.ToString()).toString);
                            commands.Add(new RBCommand("PRINT motor: " + (rb.axesAngle[motor_nb] * 10).ToString()).toString);
                            commands.Add(new RBCommand(Config.RB_COMMAND_SET, 'M', motor_nb, _RB_id, "pa", (rb.axesAngle[motor_nb] * 10).ToString()).toString);
                            if (_delta.Any()) //update my delta 
                            {
                                _delta[n - 1][motor_nb] = _theta_brs[motor_nb] - rb.axesAngle[motor_nb];
                                commands.Add(new RBCommand("PRINT delta update: " + (_delta[n - 1][motor_nb]).ToString()).toString);
                            }

                        }

                        else  //probably command not received so send again. 
                        {
                            command = new RBCommand(Config.RB_COMMAND_ASK, 'M', motor_nb, _RB_id, "x", "");//ask motor position 
                            commands.Add(command.toString);
                            commands.Add(_last_command);
                            commands.Add(new RBCommand("PRINT timeout " + _last_command).toString);
                        }

                    }
                    break;

                case process_listOfCommands:
                    mytime = Time.time;
                    if (listOfCommands.Any())
                    {
                        _last_command = listOfCommands[0];
                        commands.Add(listOfCommands[0]);
                        listOfCommands.RemoveAt(0);
                        _task = wait;
                        _task_after_wait = process_listOfCommands;
                        blocked_counter = 0;
                    }
                    else
                        _task = ask_acm;
                    break;

                case start_align:
                    align_trial++;
                    var increments = new List<int>(Enumerable.Range(0, 26)); //numbers from 0 to 25 to account for the 26 different combinations.  


                    mytime = Time.time;
                    commands.Add(new RBCommand("PRINT Start Align").toString);

                    if (first_trial)
                    {
                        _task = check_other_acm;
                        first_trial = false;
                        increments.Shuffle();

                        int index = _f.IndexOf(_f.Last());
                        for (int i = 0; i < 3; i++)
                        {
                            _theta_brs[i] = _theta_visited[index][i];
                        }
                        f_min = _f[index];
                        _f.Clear();
                        _f.Add(f_min);
                        try_cases = 0;
                        _delta.Clear();
                        _delta.Add(_theta_brs);

                        command = new RBCommand("PRINT (f IN FIRST TRIAL,theta_brs) : " + _f.ToString() + "   " + _theta_brs[0].ToString() + "    " + _theta_brs[1].ToString() + "    " + _theta_brs[2].ToString());
                        commands.Add(command.toString);
                        n = 1;

                    }
                    if (first_trial == false)
                    {


                        if (n > N - 1)
                        {
                            n = 0;
                            for (int i = 0; i < _delta.Count(); i++)
                            {
                                command = new RBCommand("PRINT Delta dans n > N -1: " + (_delta[i][0]).ToString() + "  " + (_delta[i][1]).ToString() + "  " + (_delta[i][2]).ToString() + "  ");
                                commands.Add(command.toString);
                            }

                            if (_f.Any())
                            {
                                int index = _f.IndexOf(_f.Min());
                                for (int i = 0; i < 3; i++)
                                {
                                    _theta_brs[i] = _delta[index][i];//limit_angle(_theta_brs[i] + _delta[index][i] + _delta[index][i], _angle_limits[i] + max_randm, _angle_limits[i] - max_randm);
                                    //_theta_desired[i] = _theta_brs[i];
                                }


                                for (int i = 0; i < N; i++)
                                {
                                    command = new RBCommand("PRINT (f,delta) : " + _f[i].ToString() + "   " + _delta[i][0].ToString() + "    " + _delta[i][1].ToString() + "    " + _delta[i][2].ToString());
                                    commands.Add(command.toString);
                                }

                                f_min = _f[index];
                                _f.Clear();
                                _delta.Clear();

                                if (!ignore_best)
                                {
                                    _f.Add(f_min);
                                    _delta.Add(_theta_brs);
                                    n = 1;
                                }
                                else
                                    ignore_best = false;


                                if (index != 0)
                                {
                                    increments.Shuffle();
                                    try_cases = 0;
                                    STD[0] = 1;
                                    STD[1] = 1;
                                    STD[2] = 1;
                                    track_multiplier = 0;
                                }


                            }
                        }
                        if (try_cases == 25)
                        {
                            /* if (track_multiplier >= 7)
                                 track_multiplier = 0;
                             STD[0] = std_multiplier[track_multiplier, 0];
                             STD[1] = std_multiplier[track_multiplier, 1];
                             STD[2] = std_multiplier[track_multiplier, 2];
                             increments.Shuffle();
                             try_cases = 0;
                             track_multiplier++;*/
                            ignore_best = true;

                        }


                        bool contains = true;
                        float[] theta_now = new float[] { 0.0f, 0.0f, 0.0f };

                        while (contains)
                        {
                            command = new RBCommand("PRINT IN WHILE");
                            commands.Add(command.toString);



                            for (int i = 0; i < 3; i++)
                            {
                                _theta_desired[i] = limit_angle(_theta_brs[i] + STD[i] * increment_table[increments[try_cases], i], _angle_limits[i] + max_randm, _angle_limits[i] - max_randm);
                                theta_now[i] = _theta_desired[i];
                            }

                            //contains = _theta_visited.Contains(theta_now);  //COMMENT

                            command = new RBCommand("PRINT theta_now before contains" + (theta_now[0]).ToString() + "  " + (theta_now[1]).ToString() + "  " + (theta_now[2]).ToString() + "  ");
                            commands.Add(command.toString);

                            //contains = _theta_visited.Any(element => element == theta_now);

                            //contains check:
                            bool match = false;
                            bool close_enough = false;

                            // FIRST CONDITION TO MATCH, NOT BEING SEEN BEFORE
                            foreach (float[] item in _theta_visited)
                            {
                                //command = new RBCommand("PRINT theta_now" + (theta_now[0]).ToString() + "  " + (theta_now[1]).ToString() + "  " + (theta_now[2]).ToString() + "  ");
                                //commands.Add(command.toString);
                                // command = new RBCommand("PRINT item" + (item[0]).ToString() + "  " + (item[1]).ToString() + "  " + (item[2]).ToString() + "  ");
                                //commands.Add(command.toString);

                                if (item[0] == theta_now[0] && item[1] == theta_now[1] && item[2] == theta_now[2])
                                {
                                    match = true;
                                    command = new RBCommand("PRINT THETA ALREADY VISITED");
                                    commands.Add(command.toString);

                                    command = new RBCommand("PRINT match" + match.ToString());
                                    commands.Add(command.toString);
                                    break;
                                }


                            }

                            //SECOND CONDITION TO MATCH (NB OF COMMAND FROM BASELINE)
                            float nb_com = Math.Abs(theta_now[0] - _theta_baseline[0]) + Math.Abs(theta_now[1] - _theta_baseline[1]) + Math.Abs(theta_now[2] - _theta_baseline[2]);
                            command = new RBCommand("PRINT nb_com " + nb_com.ToString());
                            commands.Add(command.toString);

                            if (nb_com < 4) //To change for docking in case face to face
                            {
                                close_enough = true;

                            }

                            command = new RBCommand("PRINT close_enough " + close_enough.ToString());
                            commands.Add(command.toString);

                            // command = new RBCommand("PRINT match after foreach" + match.ToString());
                            //commands.Add(command.toString);

                            if (match == false && close_enough == true)
                            {
                                contains = false;
                            }

                            //contains = _theta_visited.Any(p => p.SequenceEqual(theta_now)); //NOT COMMENT

                            command = new RBCommand("PRINT theta_now after contains" + (theta_now[0]).ToString() + "  " + (theta_now[1]).ToString() + "  " + (theta_now[2]).ToString() + "  ");
                            commands.Add(command.toString);



                            command = new RBCommand("PRINT theta_size, contains? : " + (_theta_visited.Count()).ToString() + "  " + contains.ToString());
                            commands.Add(command.toString);
                            try_cases++;

                            if (try_cases == 25)
                            {
                                if (track_multiplier >= 7)
                                    track_multiplier = 0;
                                STD[0] = std_multiplier[track_multiplier, 0];
                                STD[1] = std_multiplier[track_multiplier, 1];
                                STD[2] = std_multiplier[track_multiplier, 2];
                                increments.Shuffle();
                                try_cases = 0;
                                track_multiplier++;
                                //ignore_best = true;

                            }





                        }

                        for (int i = 0; i < _theta_visited.Count(); i++)
                        {
                            command = new RBCommand("PRINT theta_visited" + (_theta_visited[i][0]).ToString() + "  " + (_theta_visited[i][1]).ToString() + "  " + (_theta_visited[i][2]).ToString() + "  ");
                            commands.Add(command.toString);
                        }


                        _delta.Add(theta_now);

                        for (int i = 0; i < _delta.Count(); i++)
                        {
                            command = new RBCommand("PRINT Delta:" + (_delta[i][0]).ToString() + "  " + (_delta[i][1]).ToString() + "  " + (_delta[i][2]).ToString() + "  ");
                            commands.Add(command.toString);
                        }

                        command = new RBCommand("PRINT increment: " + (STD[0] * increment_table[increments[try_cases], 0]).ToString() + "    " + (STD[1] * increment_table[increments[try_cases], 1]).ToString() + "    " + (STD[2] * increment_table[increments[try_cases], 2]).ToString());
                        commands.Add(command.toString);
                        command = new RBCommand("PRINT theta: " + _theta_desired[0].ToString() + "    " + _theta_desired[1].ToString() + "  " + _theta_desired[2].ToString());
                        commands.Add(command.toString);
                        command = new RBCommand("PRINT delta: " + _delta[n][0].ToString() + "    " + _delta[n][1].ToString() + "  " + _delta[n][2].ToString());
                        commands.Add(command.toString);
                        command = new RBCommand("PRINT (n,try_cases):  " + n.ToString() + "  " + try_cases.ToString());
                        commands.Add(command.toString);
                        n++;
                        _task = fill_list;
                    }
                    mytime = Time.time;
                    if (align_trial >= 100)
                    {
                        for (int i = 0; i < 3; i++)
                            _theta_desired[i] = _theta_brs[i];

                    }
                    break;

                case quit:
                    commands.Add(new RBCommand("PRINT ACM Aligned: Plugin exit after " + align_trial.ToString() + " trials.").toString);

                    // commands.Add(new RBCommand("EXIT").toString);
                    _task = done;
                    break;
                case done:
                    break;

            }



            //// END PROCESS ////

            return commands;
        }


        /// <summary>
        /// Called when a key is pressed in the GUI
        /// </summary>
        /// <param name="keycode">The code of the key pressed : https://docs.unity3d.com/ScriptReference/KeyCode.html </param>
        /// <returns>The list of commands to execute in the GUI and/or send to the Roombots modules</returns>
        public static List<string> receiveKeyDown(KeyCode keycode)
        {
            List<string> commands = new List<string>();
            RBCommand command = null;

            RBStatus rb = Plugin._rbList[_RB_id];
            switch (keycode)
            {
                case KeyCode.DownArrow: //ACM1 close

                    float newpos1 = Saturation(rb.acmPosition[1] + 10);
                    command = new RBCommand("PRINT move ACM1 to " + newpos1);
                    commands.Add(command.toString);
                    command = new RBCommand(Config.RB_COMMAND_SET, 'X', 1, _RB_id, "pos", (newpos1).ToString());
                    commands.Add(command.toString);

                    break;


                case KeyCode.A: //ACC values

                    command = new RBCommand("PRINT accel _ACM0: " + rb.acmAcc[1 - _ACM][0].ToString() + "  " + rb.acmAcc[1 - _ACM][1].ToString() + "  " + rb.acmAcc[1 - _ACM][2].ToString());
                    commands.Add(command.toString);

                    command = new RBCommand("PRINT accel _ACM1: " + rb.acmAcc[_ACM][0].ToString() + "  " + rb.acmAcc[_ACM][1].ToString() + "  " + rb.acmAcc[_ACM][2].ToString());
                    commands.Add(command.toString);

                    pitch0 = 180 * Math.Atan2(-(rb.acmAcc[1 - _ACM][2] - 2048), Math.Sqrt((double)(Math.Pow(rb.acmAcc[1 - _ACM][0] - 2048, 2) + Math.Pow(rb.acmAcc[1 - _ACM][1] - 2048, 2)))) / Math.PI;
                    pitch1 = 180 * Math.Atan2(-(rb.acmAcc[_ACM][2] - 2048), Math.Sqrt((double)(Math.Pow(rb.acmAcc[_ACM][0] - 2048, 2) + Math.Pow(rb.acmAcc[_ACM][1] - 2048, 2)))) / Math.PI;

                    command = new RBCommand("PRINT calculated pitch0 is: " + pitch0.ToString());
                    commands.Add(command.toString);

                    command = new RBCommand("PRINT calculated pitch1 is: " + pitch1.ToString());
                    commands.Add(command.toString);



                    _task = done;
                    break;


                case KeyCode.UpArrow:   // ACM1 open

                    float newpos11 = Saturation(rb.acmPosition[1] - 10);
                    command = new RBCommand("PRINT move ACM1 to " + newpos11);
                    commands.Add(command.toString);
                    command = new RBCommand(Config.RB_COMMAND_SET, 'X', 1, _RB_id, "pos", (newpos11).ToString());
                    commands.Add(command.toString);
                    break;

                case KeyCode.C:

                    command = new RBCommand("PRINT cur cost norm hall: " + (rb.acmHall[_ACM][0] + rb.acmHall[_ACM][1] + rb.acmHall[_ACM][2] + rb.acmHall[_ACM][3]) / normal_hall);
                    commands.Add(command.toString);
                    command = new RBCommand("PRINT cur norm ir: " + ((rb.acmIR[_ACM][0] + rb.acmIR[_ACM][1]) / normal_IR));
                    commands.Add(command.toString);
                    break;

                case KeyCode.LeftArrow:

                    // Ask the sensors' values of the ACM0 
                    //rb = Plugin._rbList[_RB_id];
                    command = new RBCommand(Config.RB_COMMAND_ASK, 'P', 0, _RB_id, "x", "");
                    commands.Add(command.toString);
                    //ask ir
                    command = new RBCommand(Config.RB_COMMAND_ASK, 'P', 0, _RB_id, "ir", "");
                    commands.Add(command.toString);

                    //ask hall
                    command = new RBCommand(Config.RB_COMMAND_ASK, 'P', 0, _RB_id, "h", "");
                    commands.Add(command.toString);
                    //ask acc
                    command = new RBCommand("PRINT accel are:" + rb.acmAcc[0][0].ToString() + " " + rb.acmAcc[0][1].ToString() + "" + rb.acmAcc[0][2].ToString());
                    commands.Add(command.toString);

                    break;

                case KeyCode.RightArrow:
                    // Ask the sensors' values of the ACM1
                    //rb = Plugin._rbList[_RB_id];
                    command = new RBCommand(Config.RB_COMMAND_SET, 'L', 1, _RB_id, "c", "r");
                    commands.Add(command.toString);

                    // Ask the sensors' values of the ACM1
                    //rb = Plugin._rbList[_RB_id];
                    command = new RBCommand(Config.RB_COMMAND_ASK, 'P', 1, _RB_id, "x", "");
                    commands.Add(command.toString);
                    //ask ir
                    /*
                    command = new RBCommand(Config.RB_COMMAND_ASK, 'P', 1, _RB_id, "ir", "");
                    commands.Add(command.toString);

                    //ask hall

                    command = new RBCommand(Config.RB_COMMAND_ASK, 'P', 1, _RB_id, "h", "");
                    commands.Add(command.toString);
                    //ask acc
                    command = new RBCommand("PRINT accel are:" + rb.acmAcc[_ACM][0].ToString() + "  " + rb.acmAcc[_ACM][1].ToString() + "  " + rb.acmAcc[_ACM][2].ToString());
                    commands.Add(command.toString);
                    */
                    break;


                case KeyCode.Q:               // Stop the execution of the plugin
                    commands.Add(new RBCommand("PRINT Plugin exit").toString);
                    commands.Add(new RBCommand("EXIT").toString);
                    break;


                //// END PROCESS ////

                default:
                    break;
            }

            return commands;
        }



        /// <summary>
        /// Called when the value of an axis of a joystick is changed in the GUI
        /// </summary>
        /// <returns>The list of commands to execute in the GUI and/or send to the Roombots modules</returns>
        public static List<string> receiveJoystickAxes(Dictionary<string, float> axesValuesChanged)
        {
            List<string> commands = new List<string>();
            RBCommand command = null;

            foreach (string axisName in axesValuesChanged.Keys)
            {
                float axisValue = axesValuesChanged[axisName];

                switch (axisName)
                {

                    //// TODO PROCESS receiveJoystickAxes ////


                    case "JoystickAxis1":        // speed Motor1 +-900
                        string m1Speed = "";
                        if (axisValue < 0)
                            m1Speed = "900";
                        else if (axisValue == 0)
                            m1Speed = "0";
                        else
                            m1Speed = "-900";

                        command = new RBCommand(Config.RB_COMMAND_SET, 'M', 1, _RB_id, "sa", m1Speed);
                        commands.Add(command.toString);
                        break;

                    case "JoystickAxis2":         // speed Motor0 +-1200
                        string m0Speed = "";
                        if (axisValue < 0)
                            m0Speed = "-1200";
                        else if (axisValue == 0)
                            m0Speed = "0";
                        else
                            m0Speed = "1200";

                        command = new RBCommand(Config.RB_COMMAND_SET, 'M', 0, _RB_id, "sa", m0Speed);
                        commands.Add(command.toString);
                        break;

                    case "JoystickAxis3":         // speed Motor2 +-1200
                        string m2Speed = "";
                        if (axisValue < 0)
                            m2Speed = "-1200";
                        else if (axisValue == 0)
                            m2Speed = "0";
                        else
                            m2Speed = "1200";

                        command = new RBCommand(Config.RB_COMMAND_SET, 'M', 2, _RB_id, "sa", m2Speed);
                        commands.Add(command.toString);
                        break;


                    //// END PROCESS ////

                    default:
                        break;
                }
            }

            return commands;
        }

        /// <summary>
        /// saturation function for acm positions. 
        /// </summary>
        /// <returns> saturated value if value is outside the acm pos limits. </returns>
        private static float Saturation(float value)
        {
            if (value >= Config.RB_ACM_MIN_POS && value <= Config.RB_ACM_MAX_POS)
                return value;
            else if (value > Config.RB_ACM_MAX_POS)
                return Config.RB_ACM_MAX_POS;
            else
                return Config.RB_ACM_MIN_POS;

        }
    }
    public static class ThreadSafeRandom
    {
        [ThreadStatic] private static System.Random Local;

        public static System.Random ThisThreadsRandom
        {
            get { return Local ?? (Local = new System.Random(unchecked(Environment.TickCount * 31 + Thread.CurrentThread.ManagedThreadId))); }
        }
    }

    static class MyExtensions
    {
        public static void Shuffle<T>(this IList<T> list)
        {
            int n = list.Count;
            while (n > 1)
            {
                n--;
                int k = ThreadSafeRandom.ThisThreadsRandom.Next(n + 1);
                T value = list[k];
                list[k] = list[n];
                list[n] = value;
            }
        }
    }
}