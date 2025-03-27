#!/usr/bin/env python
import rospy
import socket
import heapq
from std_msgs.msg import *
from misty.srv import *
from misty.msg import *
from misty_reading.msg import *

from difflib import SequenceMatcher
import fuzzy
import Levenshtein

STORIES = ['Sponges',  
            'Why_We_Need_Water', 
            'Trees',
            'The_Raft',
            'Honesty',
            'Sunset_at_the_Beach', 
            'Breathing', 
            'Animal_Tools',
            'The_Sukkah_Next_Door', 
            'Helen_Keller', 
            'Sloths_and_Monkeys',
            'Changing_Bedtime',
            'Coyotes_and_Wolves',
            'Fizzy_Water', 
            'Prize_Winning_Vegetables', 
            'Crows',  
            'Government',  
            'Digital_Music_Recording']

# Pre-selected Vocaulary Indices
VOCAB = [[[27,66,74],[11,57,74],[4,46,81]],
         [[10,62,74],[14,40,46],[16,42,51]],
         [[24,30,63],[16,27,31],[12,33,44]],
         [[2,40,67],[51,56,64],[15,25,46]],
         [[10,17,75],[2,22,80],[13,52,102]],
         [[10,45,82],[36,40,57],[20,37,43]],
         [[11,22,100],[2,20,30],[4,6,70]],
         [[16,18,90],[18,36,65],[26,63,92]],
         [[10,37,86],[3,86,94],[8,34,78]],
         [[30,61,81],[3,33,82],[3,59,73]],
         [[16,31,59],[18,19,45],[21,35,41]],
         [[0,30,56],[3,62,120],[3,24,110]],
         [[0,8,60],[83,131,138],[32,50,59]],
         [[5,50,125],[37,90,98],[45,67,78]],
         [[25,52,117],[23,24,109],[63,107,117]],
         [[18,50,59],[38,68,91],[1,8,102]],
         [[57,61,75],[1,127,134],[18,70,102]],
         [[83,85,99],[1,24,63],[47,84,111]]
         ]

class CorrectionManager():

    def __init__(self):

        rospy.init_node('correction_manager')
        if len(sys.argv) <= 2:
            # Two stories must be indicated
            rospy.loginfo("Usage: rosrun misty_reading correction_manager.py <first story number> <second story number>")
            return

        # Init vars
        self.rate = rospy.Rate(1)
        self.conn = None
        self.speaking = True
        self.start = True
        self.first_story = True
        self.condition = 'robot'

        self.transcribed_text = []
        self.correction_indices = []
        self.word_selections = []
        self.page = 0
        self.text_length = 0
        
        # Load stories using user input 
        self.filepath = '/home/ubuntu/catkin_ws/src/misty_reading/texts/'
        self.story1_idx = int(sys.argv[1])-1
        self.story2_idx = int(sys.argv[2])-1

        self.story1 = STORIES[self.story1_idx] + '.txt'
        self.story2 = STORIES[self.story2_idx] + '.txt'
        self.get_story(self.story1)

        self.introduction =     """Hello there, my name is Misty. Today I'm going to listen to you while you read aloud to me.
                                We're going to read two stories together. When you're done reading a page, you can hit the done button. 
                                After each page I'll ask you to repeat some words. 
                                When you finish repeating the words, you can hit the next page button. Please begin when you're ready"""
        
        self.story_transition = """Good work reading the first story! I hope you liked it, I sure enjoyed listening to you. 
                                    We're going to read one more story and then you'll be asked to complete a short survey.
                                    Please begin when you're ready."""
        
        self.ending =           """You're all finished! Thank you for reading to me, I really enjoyed it. Please wait here for the researcher."""

        self.give_corrections =  ["Good job! I'm going to highlight some words, could you please repeat them in order? Once you're finished, you can hit the next page button",
                                    "Can you please repeat the highlighted words?"]

        self.page_transition = """Nice work! Let's keep going"""

        # Initialize publishers and subscriber
        self.move_arm = rospy.Publisher('misty/command/moveArm', Int32MultiArray, queue_size=10)
        self.move_head = rospy.Publisher('misty/command/moveHead', Int32MultiArray, queue_size=10)
        self.speak_pub = rospy.Publisher('misty/command/tts', SpeechInfo, queue_size=10)
        self.face_change = rospy.Publisher('misty/command/changeFacialImage', String, queue_size = 10)
        self.correction_pub = rospy.Publisher('corrections', String, queue_size = 10)
        self.stories_pub = rospy.Publisher('stories', String, queue_size = 10)

        self.transcript_sub = rospy.Subscriber('transcript', Transcription, self.transcript_callback)
        self.tablet_sub = rospy.Subscriber('tablet_msg', String, self.tablet_callback)
        self.tablet_status_sub = rospy.Subscriber('tablet_status', String, self.tablet_status_callback)
        self.word_select_sub = rospy.Subscriber('word_selections', Int32MultiArray, self.word_select_callback)

        # Sleep to give a moment for initialization
        self.rate.sleep()
        self.rate.sleep()

        # Shutdown behavior
        rospy.on_shutdown(self.shutdown)

    def get_story(self, story):
        # fetch and process story from txt files
        with open(self.filepath + story) as f:
            lines = f.readlines()

        self.story_text = []
        for l in lines:
            line = l.split()
            self.story_text.append(line)

        # reset page and text length
        self.text_length = len(self.story_text)
        self.page = 0

    def prepare_text(self, text):
        # Process text for easy comparison by removing capitalization and 
        # end punctuation (but leaving apostrophes alone, for example)
        # This is due to quirks in speech recognition that will often leave in 
        # characters e.g., '%' for the spoken word 'percent'
        text = [''.join(character for character in word
                    if character not in ['.', ',', '!', '?', '"','(',')'])
                    for word in text
                    if word]
        text = [word.lower() for word in text]
        return text
    
    def shutdown(self):
        self.reset_misty()

    def tablet_callback(self, msg):
        # get msg from tablet
        """
        if turn page - get corrections and vocab and publish results

            if end of story self.first_story = False
            dialogue to repeat words
            wait for page turn again
            new intro for second story

        This is where logic for getting corrections will go -> callback triggers get_corrections
        
        """
        msg = msg.data
        if msg == 'human':
            # Set condition to human if selected (default to robot)
            self.condition = 'human'

        if msg == "get story":
            # Tell tablet which stories to load
            self.intro_sequence()

            stories = String()
            stories.data = str([self.story1_idx, self.story2_idx])
            self.stories_pub.publish(stories)

        elif msg == "get corrections":
            # Get corrections and vocab for current page & transcribed text
            self.get_corrections(self.story_text[self.page], self.transcribed_text)
            self.get_vocabulary()
            # vocab_corrections = self.vocabulary[self.page]

            if self.condition == 'human':
                # If human, overwrite suggestions from speech recog with human selections
                self.correction_indices = self.word_selections

            # Shuffle correction indices before augmentation

            # Get relevant vocab to augment corrections
            self.correction_indices += self.vocabulary
            self.correction_indices = self.correction_indices[:3]

            # send message back to tablet node
            corrections = String()
            corrections.data = str(self.correction_indices)
            self.correction_pub.publish(corrections)

            # Trigger robot animation
            if self.condition == 'robot':
                self.correction_sequence()
        
            # Reset corrections
            self.correction_indices = []
            self.word_selections = []

        elif msg == 'next page':
            # Update page and trigger animation sequence
            if self.page == 2:
                # If end of first story, fetch second
                if self.first_story:
                    self.get_story(self.story2)

                    # Trigger robot animation
                    if self.condition =='robot':
                        self.story_transition_sequence()
                    self.first_story = False
                else:
                # Trigger ending 
                    if self.condition =='robot':
                        self.end_sequence()
            else:
                self.page += 1
                if self.condition == 'robot':
                    self.turn_page_sequence()
        
    def tablet_status_callback(self, msg):
        # Start introduction when connection achieved
        print(msg.data)
        if msg.data == 'Connected':
            # self.intro_sequence()
            pass

    def transcript_callback(self, data):
        # get corrections from recognized speech while misty is not speaking
        if not self.speaking:
            self.transcribed_text += data.word_list
            self.transcribed_text = self.prepare_text(self.transcribed_text)
            print('transcribed: '+ str(self.transcribed_text))

    def word_select_callback(self, msg):
        # get human input for misprounounced words
        self.word_selections = list(msg.data)

    def get_corrections(self, text, transcribed_text):
        '''
        Obtain words that should be repeated based on phonetic difference between the text and the 
        recognized speech.

        input:
        text [list] = ground truth text broken up into list
        sr_transcript [list] = recognized speech in a list

        out: correction_indices [list] = word indices from text that should be repeated
        '''

        dmeta = fuzzy.DMetaphone()
        seq_match = SequenceMatcher(None, text, transcribed_text)
        matching_blocks = seq_match.get_matching_blocks()
        l, m = 0, 0

        for block in matching_blocks:

            # Matching sequence start indices i and j, length k
            i, j, k = block

            txt = text[l:i].copy()
            sr = transcribed_text[m:j].copy()

            print(txt)
            print(sr)
            
            # Check for mistakes
            if len(sr) == len(txt):
                compare = list(zip(txt, sr))
                for idx, comp in enumerate(compare):

                    txt_res = dmeta(comp[0])[0]
                    sr_res = dmeta(comp[1])[0]
                    
                    if (txt_res and sr_res) is not None:
                        dist = Levenshtein.distance(txt_res, sr_res)
                        # print("Dist: " + str(dist))

                        if (dist > 1 and len(txt[idx]) > 5) or dist > 4:
                            # print('Please repeat: '+ txt[idx] + ' at global index ' + str(idx + l))
                            self.correction_indices.append(idx +l)

            # Update mismatch index
            l, m = i + k, j + k

    def get_vocabulary(self):
        if self.first_story: story = self.story1_idx
        else: story = self.story2_idx
        self.vocabulary = VOCAB[story][self.page]

    def wave(self):
        '''
        Have Misty wave
        '''
        pos1 = Int32MultiArray()

        pos1.data = [0, -60, 100]
        self.move_arm.publish(pos1)
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()

        pos1.data = [0, 30, 100]
        self.move_arm.publish(pos1)
        self.rate.sleep()

    def look_down(self):
        pos2 = Int32MultiArray()
        pos2.data = [0, 20, 0, 100] 

        self.move_head.publish(pos2)
        self.rate.sleep()
    
    def tilt_head(self):
        pos2 = Int32MultiArray()
        pos2.data = [20, 0, 0, 100] 

        self.move_head.publish(pos2)
        self.rate.sleep()


    def speak(self, text):
        msg = SpeechInfo()
        msg.text = text
        msg.flush_queue = True
        self.speak_pub.publish(msg)

    def change_face(self, expression):
        '''
        Change misty's face to one of the available images (See misty studio for file names)
        expression[str]: name of image to display
        '''
        msg = String()
        msg.data = expression        
        self.face_change.publish(msg)

    def blink(self):
        self.change_face("e_defaultcontent.jpg")
        self.change_face("e_SystemBlinkStandard.jpg")
        self.rate.sleep()
        self.change_face("e_defaultcontent.jpg")

    def correction_sequence(self):
        '''
        Misty directs child.
        '''
        self.speaking = True
        if self.start:
            self.speak(self.give_corrections[0])
            self.start = False
        else:
            self.speak(self.give_corrections[1])
        self.reset_misty()
        self.change_face("e_apprehensionconcerned.jpg")
        self.tilt_head()
        self.rate.sleep()
        self.change_face("e_defaultcontent.jpg")
        self.look_down
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.speaking = False
        self.transcribed_text = []

    def turn_page_sequence(self):
        '''
        Misty directs child.
        '''
        self.speaking = True
        self.speak(self.page_transition)
        self.reset_misty()
        self.change_face("e_apprehensionconcerned.jpg")
        self.rate.sleep()
        self.change_face("e_defaultcontent.jpg")
        self.look_down()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.speaking = False
        self.transcribed_text = []

    def story_transition_sequence(self):
        '''
        Misty directs child.
        '''
        self.speaking = True
        self.speak(self.story_transition)
        self.reset_misty()
        self.tilt_head()
        self.change_face("e_joy.jpg")
        self.rate.sleep()
        self.change_face("e_defaultcontent.jpg")
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.speaking = False
        self.transcribed_text = []

    def end_sequence(self):
        '''
        Misty directs child.
        '''
        self.speaking = True
        self.speak(self.ending)
        self.change_face("e_joy.jpg")
        self.rate.sleep()
        self.change_face("e_defaultcontent.jpg")
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.look_down()
        self.speaking = False

    def intro_sequence(self):
        '''
        Have Misty introduce herself.
        '''
        self.speaking = True
        self.speak(self.introduction)
        self.reset_misty()
        self.change_face("e_defaultcontent.jpg")
        self.tilt_head()
        self.rate.sleep()
        self.change_face("e_joy.jpg")
        self.wave()
        self.change_face("e_defaultcontent.jpg")
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.look_down()
        self.speaking = False
        self.transcribed_text = []
        
    def reset_misty(self):
        '''
        Reset Misty's posture

        '''
        pos1 = Int32MultiArray()
        pos1.data = [2, 50, 100]
        self.move_arm.publish(pos1)

        pos2 = Int32MultiArray()
        pos2.data = [0, 0, 0, 100]
        self.move_head.publish(pos2)
        self.rate.sleep()
  
if __name__ == "__main__":
    
    corr_manager = CorrectionManager()
    # corr_manager.intro_sequence()

    while not rospy.is_shutdown():
        rospy.spin()

    corr_manager.shutdown()