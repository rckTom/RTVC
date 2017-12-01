// z. B. BertOS als RTOS fuer den Arduino verwenden

static const int st_on_ramp = 1;
static const int st_in_flight = 2;
static const int st_recovery = 3;

int state;

void on_ramp() {
	// do stuff
	state = st_in_flight;
}

void in_flight() {
	// do stuff
	state = st_recovery;
}

void recovery() {
	// do stuff
	// returning to another state is not necessary

void statectrl(int current_state) {
	switch(current_state) {
		case st_on_ramp: on_ramp(); break;
		case st_in_flight: in_flight(); break;
		case st_recovery: recovery(); break;
	}
}

void main() {
	state = st_on_ramp;
	while(1) {
		statectrl(state);
	}
}