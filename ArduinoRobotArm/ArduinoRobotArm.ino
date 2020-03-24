const float L1 = 11.2;
const float L2 = 5.6;
const float L3 = 3.2;

const float ALPHA_LOWER = 0;
const float ALPHA_UPPER = 90;
const float BETA_LOWER = 50;
const float BETA_UPPER = 130;
const float GAMMA_LOWER = -60;
const float GAMMA_UPPER = 60;

const float OMEGA_LOWER = (ALPHA_UPPER - BETA_LOWER - GAMMA_LOWER) * -1;
const float OMEGA_UPPER = ALPHA_LOWER - BETA_UPPER - GAMMA_UPPER;

float alpha, beta, gamma;
void setup() {
  Serial.begin(2000000);
  long start = micros();
  calc(14.4, 3.4, 60);
  long endd = micros();
  Serial.print("setup() used ");
  Serial.print(endd - start);
  Serial.println("us");
  print_result();
}

void loop() {
}

boolean calc(float x, float y, float omega) {
  bool valid = internal_calc(x, y, omega);
  if (valid) {
    //Serial.println("first wish is valid");
    return true;
  }
  //Serial.println("first wish not valid -> brute force search of omega");
  bool valid_found = false;
  float first_valid, last_valid;
  for (int i = 0; i < 21; i++) {
    float om = map(i, 0, 20, OMEGA_LOWER, OMEGA_UPPER);
    valid = internal_calc(x, y, om);
    if (valid && !valid_found) {
      first_valid = om;
      valid_found = true;
    }
    if (valid) {
      last_valid = om;
    }
    if (valid_found && !valid) {
      break;
    }
  }
  if (valid_found) {
    float om = (first_valid + last_valid) / 2;
    //Serial.print("omega search successful, om=");
    //Serial.println(om);
    return internal_calc(x, y, om);
  } else {
    //Serial.println("omega search not successful");
  }

  //Serial.print("x and y probably too far away");
  int steps = 100;
  while (steps > 0 && !internal_calc(x, y, omega)) {
    x *= 0.9;
    y *= 0.9;
    steps--;
  }
  return internal_calc(x, y, omega);
}

boolean internal_calc(float x, float y, float omega) {
  /*Serial.println("****************Calculate Position*******************");
  Serial.print("x=");
  Serial.println(x);
  Serial.print("y=");
  Serial.println(y);
  Serial.print("omega=");
  Serial.println(omega);*/

  float x_gamma = cos(radians(omega)) * L3;
  float y_gamma = sin(radians(omega)) * L3;

  float p2_x = x - x_gamma;
  float p2_y = y + y_gamma;

  float c = sqrt(p2_x * p2_x + p2_y * p2_y);
  float u = degrees(atan(p2_y / p2_x));

  alpha = u + degrees(acos((c * c + L1 * L1 - L2 * L2) / (2 * c * L1)));
  beta = 180 - degrees(acos((L1 * L1 + L2 * L2 - c * c) / (2 * L1 * L2)));
  gamma = omega - alpha - beta;

  if (isnan(alpha) || ALPHA_LOWER > alpha || alpha > ALPHA_UPPER) {
    /*Serial.print("alpha is invalid =");
    Serial.println(alpha);*/
    return false;
  } else if (isnan(beta) || BETA_LOWER > beta || beta > BETA_UPPER) {
    /*Serial.print("beta is invalid =");
    Serial.println(beta);*/
    return false;
  } else if (isnan(gamma) || GAMMA_LOWER > gamma || gamma > GAMMA_UPPER) {
    /*Serial.print("gamma is invalid =");
    Serial.println(gamma);*/
    return false;
  }

  /*Serial.print("x_gamma=");
  Serial.println(x_gamma);
  Serial.print("y_gamma=");
  Serial.println(y_gamma);

  Serial.print("p2_x=");
  Serial.println(p2_x);
  Serial.print("p2_y=");
  Serial.println(p2_y);

  Serial.print("c=");
  Serial.println(c);
  Serial.print("u=");
  Serial.println(u);*/

  

  return true;
}

void print_result() {
  Serial.print("alpha=");
  Serial.println(alpha);
  Serial.print("beta=");
  Serial.println(beta);
  Serial.print("gamma=");
  Serial.println(gamma);
}
