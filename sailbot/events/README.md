# Events
All event information can be found [here](https://docs.google.com/presentation/d/1Wv22dM5A8v-HcMtH69Mc2jT05uNU04fFo3Ql1LSAIbU/edit#slide=id.g2430a897f0f_0_7).

Event code is standardized to maintain compatibility with main. Events only have one responsibility: tell boatMain which waypoint to go to using next_gps().

Additional signals exist for more complex behavior:
- Return None to tell the boat to stop
- Return an EventFinished Exception to signal that the event is done and that its safe to return to RC