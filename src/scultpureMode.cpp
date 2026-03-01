#include "scultpureMode.h"
#include <Arduino.h> // Add this line to include the Arduino library
#include "config.h"
// #include "SPIFFS.h"
#include <string>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <CuteBuzzerSounds.h>
#include <vector>
// #include <map>

TMC2209Stepper driver(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS1);
TMC2209Stepper driver2(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS2);
AccelStepper stepper1(AccelStepper::DRIVER, MOTOR_0_STEP_PIN, MOTOR_0_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR_1_STEP_PIN, MOTOR_1_DIR_PIN);

const char *games[] = {
    "1. Ng1-f3 d7-d5 2. d2-d4 e7-e6 3. g2-g3 c7-c5 4. Bf1-g2 Nb8-c6 5. O-O Ng8-f6 6. c2-c4 d5xc4 7. Nf3-e5 Bc8-d7 8. Nb1-a3 c5xd4 9. Na3xc4 Bf8-c5 10. Qd1-b3 O-O 11. Qb3xb7 Nc6xe5 12. Nc4xe5 Ra8-b8 13. Qb7-f3 Bc5-d6 14. Ne5-c6 Bd7xc6 15. Qf3xc6 e6-e5 16. Ra1-b1 Rb8-b6 17. Qc6-a4 Qd8-b8 18. Bc1-g5 Bd6-e7 19. b2-b4 Be7xb4 20. Bg5xf6 g7xf6 21. Qa4-d7 Qb8-c8 22. Qd7xa7 Rb6-b8 23. Qa7-a4 Bb4-c3 24. Rb1xb8 Qc8xb8 25. Bg2-e4 Qb8-c7 26. Qa4-a6 Kg8-g7 27. Qa6-d3 Rf8-b8 28. Be4xh7 Rb8-b2 29. Bh7-e4 Rb2xa2 30. h2-h4 Qc7-c8 31. Qd3-f3 Ra2-a1 32. Rf1xa1 Bc3xa1 33. Qf3-h5 Qc8-h8 34. Qh5-g4 Kg7-f8 35. Qg4-c8 Kf8-g7 36. Qc8-g4 Kg7-f8 37. Be4-d5 Kf8-e7 38. Bd5-c6 Ke7-f8 39. Bc6-d5 Kf8-e7 40. Qg4-f3 Ba1-c3 41. Bd5-c4 Kh8-c8 42. Qf3-d5 Qc8-e6 43. Qd5-b5 Qe6-d7 44. Qb5-c5 Qd7-d6 45. Qc5-a7 Qd6-d7 46. Qa7-a8 Qd7-c7 47. Qa8-a3 Qc7-d6 48. Qa3-a2 f6-f5 49. Bc4xf7 e5-e4 50. Bf7-h5 Qd6-f6 51. Qa2-a3 Ke7-d7 52. Qa3-a7 Kd7-d8 53. Qa7-b8 Kd8-d7 54. Bh5-e8 Kd7-e7 55. Be8-b5 Bc3-d2 56. Qb8-c7 Ke7-f8 57. Bb5-c4 Bd2-c3 58. Kg1-g2 Bc3-e1 59. Kg2-f1 Be1-c3 60. f2-f4 e4xf3 61. e2xf3 Bc3-d2 62. f3-f4 Kf8-e8 63. Qc7-c8 Ke8-e7 64. Qc8-c5 Ke7-d8 65. Bc4-d3 Bd2-e3 66. Qc5xf5 Qf6-c6 67. Qf5-f8 Kd8-c7 68. Qf8-e7 Kc7-c8 69. Bd3-f5 Kc8-b8 70. Qe7-d8 Kb8-b7 71. Qd8-d7 Qc6xd7 72. Bf5xd7 Kb7-c7 73. Bd7-b5 1-0", //"1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 Ng8-f6 4. e4-e5 Nf6-d7 5. f2-f4 c7-c5 6. c2-c3 Nb8-c6 7. Nd2-f3 c5xd4 8. c3xd4 f7-f6 9. Bf1-d3 Bf8-b4+ 10. Bc1-d2 Qd8-b6 11. Ng1-e2 f6xe5 12. f4xe5 O-O 13. a2-a3 Bb4-e7 14. Qd1-c2 Rf8xf3 15. g2xf3 Nc6xd4 16. Ne2xd4 Qb6xd4 17. O-O-O Nd7xe5 18. Bd3xh7+ Kg8-h8 19. Kc1-b1 Qd4-h4 20. Bd2-c3 Be7-f6 21. f3-f4 Ne5-c4 22. Bc3xf6 Qh4xf6 23. Bh7-d3 b7-b5 24. Qc2-e2 Bc8-d7 25. Rh1-g1 Bd7-e8 26. Rd1-e1 Be8-f7 27. Rg1-g3 Ra8-c8 28. Re1-g1 Nc4-d6 29. Rg3xg7 Nd6-f5 30. Rg7-g5 Rc8-c7 31. Bd3xf5 e6xf5 32. Rg5-h5+ 1-0",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 c7-c5 4. e4xd5 Qd8xd5 5. Ng1-f3 c5xd4 6. Bf1-c4 Qd5-d6 7. O-O Ng8-f6 8. Nd2-b3 Nb8-c6 9. Nb3xd4 Nc6xd4 10. Nf3xd4 a7-a6 11. Nd4-f3 b7-b5 12. Bc4-d3 Bc8-b7 13. a2-a4 Nf6-g4 14. Rf1-e1 Qd6-b6 15. Qd1-e2 Bf8-c5 16. Re1-f1 b5-b4 17. h2-h3 Ng4-f6 18. Bc1-g5 Nf6-h5 19. Bg5-e3 Bc5xe3 20. Qe2xe3 Qb6xe3 21. f2xe3 Nh5-g3 22. Rf1-e1 Ng3-e4 23. Nf3-e5 Ne4-c5 24. Bd3-c4 Ke8-e7 25. a4-a5 Rh8-d8 26. Re1-d1 Ra8-c8 27. b2-b3 Rc8-c7 28. Rd1xd8 Ke7xd8 29. Ne5-d3 Nc5xd3 30. Bc4xd3 Rc7-c5 31. Ra1-a4 Kd8-c7 32. Kg1-f2 g7-g6 33. g2-g4 Bb7-c6 34. Ra4xb4 Rc5xa5 35. Rb4-f4 f7-f5 36. g4-g5 Ra5-d5 37. Rf4-h4 Rd5-d7 38. Bd3xa6 Rd7-d2+ 39. Kf2-e1 Rd2xc2 40. Rh4xh7+ Kc7-d6 41. Ba6-c4 Bc6-d5 42. Rh7-g7 Rc2-h2 43. Rg7xg6 Rh2xh3 44. Ke1-d2 Rh3-g3 45. Rg6-g8 Bd5xc4 46. b3xc4 Kd6-c5 47. g5-g6 Kc5-d6 48. c4-c5+ Kd6-c7 49. g6-g7 Kc7-b7 50. c5-c6+ 1-0",
    "1. e2-e4 c7-c6 2. c2-c4 d7-d5 3. e4xd5 c6xd5 4. c4xd5 Ng8-f6 5. Nb1-c3 g7-g6 6. Bf1-c4 Bf8-g7 7. Ng1-f3 O-O 8. O-O Nb8-d7 9. d2-d3 Nd7-b6 10. Qd1-b3 Bc8-f5 11. Rf1-e1 h7-h6 12. a2-a4 Nf6-d7 13. Bc1-e3 a7-a5 14. Nf3-d4 Nb6xc4 15. d3xc4 Nd7-c5 16. Qb3-a3 Nc5-d3 17. Nd4xf5 g6xf5 18. Re1-d1 Nd3-e5 19. b2-b3 Ne5-g4 20. Qa3-c1 f5-f4 21. Be3-d4 Bg7xd4 22. Rd1xd4 e7-e5 23. Rd4-d2 Qd8-h4 24. h2-h3 Ng4-f6 25. Qc1-e1 Qh4-g5 26. Nc3-e4 Nf6xe4 27. Qe1xe4 f7-f5 28. Qe4xe5 Ra8-e8 29. h3-h4 Qg5xh4 30. Qe5-c3 Re8-e4 31. d5-d6 Qh4-g5 32. f2-f3 Re4-e3 33. Qc3xa5 Rf8-e8 34. Rd2-f2 Qg5-f6 35. Ra1-d1 Re3-e5 36. d6-d7 1-0",
    "1. e2-e4 c7-c6 2. c2-c4 d7-d5 3. e4xd5 c6xd5 4. c4xd5 Ng8-f6 5. Nb1-c3 Nf6xd5 6. d2-d4 Nb8-c6 7. Ng1-f3 e7-e6 8. Bf1-d3 Bf8-e7 9. O-O O-O 10. Rf1-e1 Be7-f6 11. Bd3-e4 Nc6-e7 12. a2-a3 Ra8-b8 13. h2-h4 b7-b6 14. Qd1-d3 g7-g6 15. h4-h5 Bc8-b7 16. Bc1-h6 Rf8-e8 17. Ra1-c1 Nd5xc3 18. b2xc3 Bb7xe4 19. Re1xe4 Ne7-f5 20. Bh6-f4 Rb8-c8 21. h5xg6 h7xg6 22. Bf4-e5 Bf6xe5 23. Nf3xe5 Qd8-g5 24. Rc1-d1 Kg8-g7 25. Qd3-f3 Re8-h8 26. Re4-g4 Qg5-h5 27. Kg1-f1 Nf5-h6 28. Rg4-f4 Qh5xf3 29. Rf4xf3 Rh8-d8 30. Rf3-d3 f7-f6 31. Ne5-f3 Rd8-d5 32. Rd1-c1 Nh6-f5 33. Kf1-e2 Nf5-d6 34. Nf3-d2 Nd6-b5 35. Ke2-d1 e6-e5 36. a3-a4 Nb5-d6 37. Rc1-b1 e5-e4 38. Rd3-e3 f6-f5 39. g2-g3 g6-g5 40. Rb1-b4 Kg7-g6 41. Kd1-c2 Rc8-c6 42. Re3-e1 0-1",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 a7-a6 6. Bc1-g5 e7-e6 7. f2-f4 Bf8-e7 8. Qd1-f3 Qd8-c7 9. O-O-O Nb8-d7 10. Bf1-d3 b7-b5 11. Rh1-e1 Bc8-b7 12. Qf3-g3 b5-b4 13. Nc3-d5 e6xd5 14. e4-e5 d6xe5 15. f4xe5 O-O-O 16. Nd4-f5 Be7-c5 17. e5xf6 Qc7xg3 18. h2xg3 g7xf6 19. Bg5-f4 Nd7-e5 20. Bf4xe5 f6xe5 21. Re1xe5 Rd8-e8 22. Re5xe8+ Rh8xe8 23. Rd1-h1 Re8-h8 24. Rh1-h6 a6-a5 25. Kc1-d2 Kc8-b8 26. Rh6-f6 Rh8-f8 27. Rf6-h6 Rf8-h8 28. Rh6-f6 1/2-1/2",
    "1. e2-e4 c7-c5 2. Ng1-f3 Nb8-c6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 d7-d6 6. Bc1-e3 Nf6-g4 7. Be3-g5 Qd8-b6 8. Bf1-b5 Bc8-d7 9. O-O Qb6xd4 10. Bb5xc6 Qd4xd1 11. Bc6xd7+ Ke8xd7 12. Ra1xd1 g7-g6 13. h2-h3 Ng4-e5 14. Nc3-d5 Ne5-c6 15. b2-b4 h7-h6 16. Bg5-h4 f7-f5 17. f2-f4 Rh8-g8 18. b4-b5 Nc6-a5 19. e4-e5 Na5-c4 20. Rd1-d4 Ra8-c8 21. e5-e6+ Kd7-e8 22. b5-b6 a7xb6 23. Rf1-b1 g6-g5 24. Rb1-b4 b6-b5 25. Rb4xb5 Bf8-g7 26. Rd4xc4 Rc8xc4 27. Rb5xb7 Bg7-d4+ 28. Bh4-f2 Bd4xf2+ 29. Kg1xf2 Ke8-f8 30. Rb7xe7 Rg8-g7 31. Re7-d7 Rc4xc2+ 32. Kf2-f3 Rg7-g6 33. Rd7-d8+ Kf8-g7 34. e6-e7 g5-g4+ 35. h3xg4 f5xg4+ 36. Kf3-g3 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 g7-g6 6. Bf1-e2 Bf8-g7 7. O-O O-O 8. Bc1-g5 Nb8-c6 9. Nd4-b3 a7-a6 10. Kg1-h1 b7-b5 11. a2-a3 Bc8-b7 12. f2-f4 Rf8-e8 13. Be2-f3 h7-h6 14. Bg5-h4 Nf6-d7 15. Qd1-e1 b5-b4 16. a3xb4 Nc6xb4 17. Rf1-f2 a6-a5 18. Bf3-g4 Bb7-c6 19. Qe1-d2 Nd7-b6 20. f4-f5 g6-g5 21. Bh4-g3 Nb6-c4 22. Qd2-c1 a5-a4 1/2-1/2",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4xd5 c6xd5 4. c2-c4 Ng8-f6 5. Nb1-c3 Nb8-c6 6. Bc1-g5 Qd8-a5 7. Bg5xf6 e7xf6 8. c4xd5 Bf8-b4 9. Qd1-d2 Bb4xc3 10. b2xc3 Qa5xd5 11. Ng1-e2 O-O 12. Ne2-f4 Qd5-a5 13. Bf1-d3 Rf8-e8+ 14. Nf4-e2 Bc8-g4 15. f2-f3 Bg4-f5 16. O-O Ra8-d8 17. Bd3xf5 Qa5xf5 18. Ne2-g3 Qf5-a5 19. Rf1-e1 Re8xe1+ 20. Ra1xe1 h7-h5 21. Ng3-f1 g7-g6 22. Re1-b1 b7-b6 23. Qd2-c2 Rd8-c8 24. Qc2-b3 Qa5-f5 25. Nf1-e3 Qf5-g5 26. Ne3-c4 Qg5-f4 27. Rb1-e1 Qf4-c7 28. d4-d5 Nc6-b8 29. Nc4-d2 Qc7-c5+ 30. Kg1-f1 f6-f5 31. c3-c4 Nb8-d7 32. Qb3-e3 Qc5-c7 33. Qe3-e7 Qc7-d8 34. Qe7xd8+ Rc8xd8 35. Nd2-b3 Kg8-f8 36. Nb3-d4 Nd7-b8 37. Kf1-e2 Rd8-e8+ 38. Ke2-d2 Re8-c8 39. Kd2-d3 Rc8-c7 40. Nd4-b5 Rc7-d7 41. Nb5-d4 Rd7-c7 42. Re1-e2 Rc7-d7 43. Kd3-c3 Rd7-e7 44. Re2xe7 Kf8xe7 45. Kc3-b4 Ke7-d7 46. a2-a3 Nb8-a6+ 47. Kb4-c3 Na6-c5 48. Nd4-b3 Nc5-a6 49. a3-a4 Na6-c7 50. Kc3-d4 Kd7-d6 51. a4-a5 Nc7-a6 52. a5xb6 a7xb6 53. Nb3-c1 Na6-b4 54. Kd4-c3 Nb4-a6 55. Kc3-d4 Na6-b4 56. Kd4-c3 Nb4-a6 57. Nc1-d3 Na6-b8 58. Kc3-b4 Nb8-d7 59. Kb4-b5 Kd6-c7 60. f3-f4 Kc7-d6 61. Nd3-e5 Nd7xe5 62. f4xe5+ Kd6-c7 63. e5-e6 f7xe6 64. d5xe6 f5-f4 65. e6-e7 Kc7-d7 66. Kb5xb6 h5-h4 67. h2-h3 Kd7xe7 68. c4-c5 g6-g5 69. c5-c6 g5-g4 70. c6-c7 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Ba4xc6 d7xc6 7. d2-d3 Nf6-d7 8. b2-b3 O-O 9. Bc1-b2 f7-f6 10. Nb1-d2 Nd7-c5 11. Rf1-e1 Nc5-e6 12. Nd2-c4 c6-c5 13. Nc4-e3 Ne6-d4 14. Nf3-h4 g7-g6 15. c2-c3 Nd4-c6 16. Nh4-f3 Bc8-e6 17. Qd1-e2 Qd8-d7 18. Ra1-d1 Ra8-d8 19. d3-d4 c5xd4 20. c3xd4 Nc6xd4 21. Nf3xd4 e5xd4 22. Bb2xd4 Qd7-b5 23. Qe2-b2 c7-c6 24. f2-f4 Be7-c5 25. f4-f5 Bc5xd4 26. Rd1xd4 Qb5-e5 27. Rd4-d2 Qe5xb2 28. Rd2xb2 Be6-c8 29. f5xg6 h7xg6 30. Ne3-c4 Rf8-e8 31. Rb2-f2 Kg8-g7 32. h2-h3 Rd8-d4 33. Re1-f1 Re8xe4 34. Rf2xf6 Bc8-f5 35. Rf6-d6 b7-b5 36. Rd6xd4 Re4xd4 37. Nc4-e5 c6-c5 38. Rf1-c1 Rd4-d2 39. Rc1xc5 Rd2xa2 40. Rc5-c7+ Kg7-f6 41. Ne5-d7+ Bf5xd7 42. Rc7xd7 Ra2-a3 43. Rd7-d6+ Kf6-g5 44. b3-b4 Ra3-a4 1/2-1/2",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Nb1-c3 Bf8-b4 4. e2-e3 b7-b6 5. Ng1-e2 Bc8-a6 6. Ne2-g3 Bb4xc3+ 7. b2xc3 d7-d5 8. c4xd5 Ba6xf1 9. Ke1xf1 Qd8xd5 10. Qd1-d3 Qd5-d7 11. e3-e4 Nb8-c6 12. Bc1-g5 h7-h6 13. Bg5xf6 g7xf6 14. Ng3-h5 O-O-O 15. Nh5xf6 Qd7-d6 16. e4-e5 Nc6xe5 17. Qd3-g3 Ne5-d7 18. Qg3xd6 c7xd6 19. Nf6xd7 Kc8xd7 20. Kf1-e2 Rh8-g8 21. g2-g3 Rd8-c8 22. Ke2-d3 Rg8-g5 23. Rh1-b1 Rg5-h5 24. Rb1-h1 Rh5-f5 25. f2-f4 h6-h5 26. Ra1-f1 Rf5-a5 27. Rf1-f2 Ra5-a3 28. Rf2-c2 Kd7-e7 29. Rh1-b1 Ra3-a5 30. Rb1-b4 Rc8-g8 31. Rb4-b1 Ke7-f6 32. Kd3-e4 h5-h4 33. Rb1-g1 h4xg3 34. h2xg3 Ra5-h5 35. g3-g4 Rh5-h3 36. g4-g5+ Kf6-e7 37. d4-d5 Rg8-c8 38. d5xe6 f7xe6 39. Ke4-d4 b6-b5 40. c3-c4 Rh3-h4 41. c4-c5 d6xc5+ 42. Rc2xc5 Rh4xf4+ 43. Kd4-e3 Rc8xc5 44. Ke3xf4 Rc5-c2 45. a2-a3 Rc2-c3 46. g5-g6 Ke7-f8 47. Rg1-d1 Rc3xa3 48. Rd1-d7 b5-b4 49. Kf4-g5 Ra3-g3+ 50. Kg5-f6 Rg3-f3+ 51. Kf6xe6 a7-a5 52. Rd7-a7 b4-b3 53. Ra7xa5 Kf8-g7 54. Ra5-b5 Kg7xg6 55. Rb5-b4 Rf3-d3 56. Ke6-e5 Kg6-g5 57. Ke5-e4 Rd3-g3 58. Rb4-b5+ Kg5-f6 1/2-1/2",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O b7-b5 6. Ba4-b3 Bf8-e7 7. d2-d4 d7-d6 8. d4xe5 d6xe5 9. Qd1-e2 Nc6-d4 10. Nf3xd4 Qd8xd4 11. c2-c3 Qd4-b6 12. Kg1-h1 O-O 13. Nb1-d2 Bc8-b7 14. Bb3-c2 c7-c5 15. Rf1-e1 Rf8-d8 16. Nd2-f1 Rd8-d7 17. Nf1-g3 g7-g6 18. f2-f4 e5xf4 19. Bc1xf4 Qb6-e6 20. Re1-f1 Ra8-d8 21. Ra1-e1 Qe6-g4 22. Qe2-f2 Nf6-h5 23. Ng3xh5 Qg4xh5 24. Bf4-g3 f7-f6 25. Bg3-f4 Qh5-g4 26. e4-e5 f6xe5 27. Bf4-h6 Qg4-e6 28. Qf2-g3 e5-e4 29. Bc2-d1 Rd7-d3 30. Bd1-g4 Qe6-d6 31. Qg3-f2 Qd6-e5 32. Qf2-f7+ Kg8-h8 33. Rf1-f5 Rd3-e3 34. Re1-f1 1-0",
    "1. e2-e4 c7-c6 2. c2-c4 d7-d5 3. c4xd5 c6xd5 4. e4xd5 Ng8-f6 5. Bf1-b5+ Nb8-d7 6. Nb1-c3 a7-a6 7. Bb5-e2 b7-b5 8. d5-d6 e7-e6 9. d2-d4 Bf8xd6 10. Be2-f3 Ra8-b8 11. Ng1-e2 O-O 12. O-O Qd8-c7 13. Ne2-g3 b5-b4 14. Nc3-a4 Bc8-b7 15. Bf3xb7 Qc7xb7 16. Bc1-g5 Qb7-d5 17. Bg5xf6 Nd7xf6 18. Qd1-d3 Rf8-d8 19. Rf1-d1 a6-a5 20. Ng3-f1 Qd5-g5 21. Ra1-c1 Nf6-d5 22. Nf1-d2 Nd5-f4 23. Qd3-f3 Qg5-b5 24. Nd2-c4 Qb5xa4 25. Nc4xd6 Rd8xd6 26. Qf3xf4 Rb8-d8 27. Qf4xd6 Qa4xd1+ 28. Rc1xd1 Rd8xd6 29. Rd1-c1 g7-g6 30. Rc1-c4 e6-e5 31. Kg1-f1 e5xd4 32. Kf1-e2 d4-d3+ 33. Ke2-d2 Rd6-d5 34. Rc4-e4 Kg8-g7 35. Re4-e3 1/2-1/2",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Ba4xc6 d7xc6 7. Rf1-e1 Nf6-d7 8. d2-d3 O-O 9. Nb1-d2 f7-f6 10. Nd2-c4 Nd7-c5 11. b2-b3 Nc5-e6 12. Nc4-e3 Ne6-d4 1/2-1/2",
    "1. c2-c4 e7-e5 2. Nb1-c3 Ng8-f6 3. g2-g3 Bf8-b4 4. Bf1-g2 O-O 5. Ng1-f3 Rf8-e8 6. O-O Bb4xc3 7. d2xc3 Nb8-c6 8. Nf3-e1 d7-d6 9. Ne1-c2 Bc8-e6 10. Nc2-e3 h7-h6 11. Ne3-d5 Nf6-h7 12. e2-e4 Nc6-e7 13. Qd1-e2 Qd8-d7 14. Rf1-d1 b7-b6 15. Bc1-e3 Ne7-g6 16. Be3-d2 Re8-f8 17. Rd1-f1 Ra8-e8 18. Ra1-d1 f7-f5 19. e4xf5 Be6xf5 20. Nd5-e3 Ng6-f4 21. g3xf4 e5xf4 22. Kg1-h1 Nh7-g5 23. Bg2-d5+ Kg8-h8 24. Rf1-g1 f4xe3 25. Bd2xe3 Ng5-h3 26. Qe2-h5 Re8xe3 27. f2xe3 Nh3-f2+ 28. Kh1-g2 Bf5-g4 29. Qh5-g6 Nf2xd1 0-1",
    "1. e2-e4 g7-g6 2. d2-d4 d7-d6 3. Nb1-c3 Bf8-g7 4. Bc1-g5 Nb8-c6 5. d4-d5 Nc6-e5 6. f2-f4 Ne5-d7 7. Ng1-f3 c7-c6 8. Qd1-d2 Nd7-c5 9. O-O-O Qd8-b6 10. e4-e5 Bc8-g4 11. Rd1-e1 O-O-O 12. d5xc6 b7xc6 13. Qd2-e3 f7-f6 14. Bg5-h4 g6-g5 15. f4xg5 f6xe5 16. Nc3-e4 Nc5xe4 17. Qe3xe4 Bg4-e6 18. b2-b3 Be6-d5 19. Qe4-a4 Rd8-f8 20. Bf1-a6+ Kc8-d7 21. Bh4-g3 e5-e4 22. Nf3-h4 Qb6-c5 23. Ba6-e2 Bd5-e6 24. Rh1-f1 Rf8xf1 25. Re1xf1 Qc5xg5+ 26. Kc1-b1 Qg5-c5 27. Bg3-f2 e4-e3 28. Be2-g4 Ng8-f6 29. Bg4xe6+ Kd7xe6 30. Bf2-g1 Ke6-f7 31. Rf1-e1 Nf6-e4 32. Qa4xe4 Qc5-c3 33. Qe4xe7+ Kf7xe7 34. Re1xe3+ Qc3xe3 35. Bg1xe3 Rh8-f8 36. Nh4-f3 a7-a6 37. Kb1-c1 Rf8-f5 38. Kc1-d2 Rf5-a5 39. a2-a4 Ra5-h5 40. Kd2-d3 Bg7-h6 41. Be3-f2 Rh5-d5+ 42. Kd3-e2 Rd5-f5 43. g2-g4 Rf5-f7 44. Nf3-d4 Ke7-d7 45. h2-h3 c6-c5 46. Nd4-f5 Bh6-f8 47. Bf2-g3 Kd7-e6 48. Ke2-d3 Rf7-b7 49. Bg3-e1 Bf8-g7 50. Kd3-c4 Bg7-e5 51. Be1-d2 Rb7-c7 52. Bd2-a5 d6-d5+ 53. Kc4-d3 c5-c4+ 54. Kd3-d2 Rc7-b7 55. b3xc4 d5xc4 56. Nf5-e3 Be5-f4 57. Kd2-e2 Bf4xe3 58. Ke2xe3 Rb7-b2 59. c2-c3 Rb2-h2 60. Ke3-d4 Rh2xh3 61. g4-g5 Rh3-g3 62. Kd4xc4 Rg3xg5 63. Ba5-d8 Rg5-g4+ 64. Kc4-c5 Rg4xa4 65. c3-c4 Ke6-d7 66. Bd8-h4 Ra4-a2 67. Kc5-b6 Ra2-b2+ 0-1",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4xd5 c6xd5 4. Bf1-d3 Nb8-c6 5. c2-c3 Ng8-f6 6. h2-h3 g7-g6 7. Ng1-f3 Bf8-g7 8. O-O O-O 9. Rf1-e1 Qd8-c7 10. Nb1-d2 Bc8-f5 11. Nd2-f1 Bf5xd3 12. Qd1xd3 a7-a6 13. Nf1-g3 e7-e6 14. Bc1-g5 b7-b5 15. Qd3-d2 Rf8-c8 16. Bg5-h6 Nc6-a5 17. h3-h4 Na5-c4 18. Qd2-c1 Bg7xh6 19. Qc1xh6 Nc4xb2 20. Nf3-e5 Qc7-d8 21. h4-h5 Ra8-a7 22. Re1-e3 Nb2-c4 23. Ne5xc4 Rc8xc4 24. Re3-f3 Nf6-g4 25. Qh6-f4 f7-f5 26. Qf4-d2 Ra7-c7 27. Ra1-e1 Rc7-c6 28. Ng3-f1 b5-b4 29. c3xb4 Rc4-c2 30. Qd2-d3 Qd8-h4 31. g2-g3 Qh4xh5 32. b4-b5 Rc2-c1 33. b5xc6 Rc1xe1 34. Kg1-g2 Re1-c1 35. Qd3-c3 Rc1xf1 36. Kg2xf1 Qh5-h1+ 37. Kf1-e2 Ng4-h2 38. c6-c7 Qh1-f1+ 39. Ke2-d2 Nh2xf3+ 40. Qc3xf3 Qf1-c4 41. Qf3-c3 Qc4xa2+ 42. Kd2-e3 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 d7-d6 6. g2-g4 h7-h6 7. Bf1-g2 a7-a6 8. O-O Qd8-c7 9. h2-h3 Nb8-c6 10. Nd4xc6 b7xc6 11. f2-f4 e6-e5 12. f4xe5 d6xe5 13. Bc1-e3 Bc8-e6 14. Qd1-e2 Bf8-e7 15. Nc3-a4 Nf6-d7 16. c2-c4 O-O 17. b2-b3 Qc7-a5 18. Qe2-f2 Be6xc4 19. Rf1-c1 Bc4-e6 20. Rc1xc6 Rf8-c8 21. Ra1-c1 Rc8xc6 22. Rc1xc6 Nd7-f6 23. Rc6-b6 Ra8-d8 24. Qf2-e2 Rd8-d6 25. Rb6-b8+ Kg8-h7 26. Bg2-f3 Rd6-c6 27. Rb8-b6 Rc6xb6 28. Be3xb6 Qa5-b5 29. Qe2xb5 a6xb5 30. Na4-c5 Nf6-d7 31. Nc5xd7 Be6xd7 32. Kg1-f2 Kh7-g6 33. Bf3-d1 Be7-a3 34. h3-h4 Bd7-c6 35. Kf2-f3 Kg6-f6 36. Bb6-d8+ Kf6-e6 37. Bd1-c2 Ke6-d6 38. Kf3-e3 Ba3-c1+ 39. Ke3-f3 b5-b4 40. Bc2-d3 Bc1-d2 41. Bd3-c4 Bc6-e8 42. Bd8-a5 Kd6-c5 43. Bc4-d5 Bd2-e1 44. h4-h5 Be1-h4 45. Ba5-c7 Bh4-f6 46. Bc7-b8 g7-g6 47. Bb8-a7+ Kc5-b5 48. Bd5-c4+ Kb5-c6 49. Ba7-e3 g6-g5 50. Bc4-d5+ Kc6-b5 51. Bd5-c4+ 1/2-1/2",
    "1. e2-e4 c7-c6 2. c2-c4 d7-d5 3. e4xd5 c6xd5 4. c4xd5 Ng8-f6 5. Bf1-b5+ Nb8-d7 6. Nb1-c3 a7-a6 7. Qd1-a4 Ra8-b8 8. Bb5xd7+ Bc8xd7 9. Qa4-b3 b7-b5 10. d2-d3 b5-b4 11. Nc3-e4 Qd8-a5 12. Ne4xf6+ g7xf6 13. Ng1-e2 Rh8-g8 14. O-O Bd7-a4 15. Qb3-c4 Rb8-d8 16. b2-b3 Ba4-b5 17. Qc4-e4 Qa5-b6 18. Bc1-e3 Qb6-b7 19. Ne2-f4 e7-e5 20. Rf1-e1 Rd8-d7 21. d3-d4 Ke8-d8 22. d4xe5 f6xe5 23. Qe4xh7 Rg8-g4 24. Qh7-h8 1-0",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-c3 d5xe4 4. Nc3xe4 Bc8-f5 5. Ne4-g3 Bf5-g6 6. h2-h4 h7-h6 7. Ng1-f3 Nb8-d7 8. h4-h5 Bg6-h7 9. Bf1-d3 Bh7xd3 10. Qd1xd3 Qd8-c7 11. Bc1-d2 O-O-O 12. O-O-O e7-e6 13. Ng3-e4 Ng8-f6 14. g2-g3 Nf6xe4 15. Qd3xe4 Nd7-f6 16. Qe4-e2 Bf8-d6 17. Kc1-b1 Kc8-b8 18. Nf3-e5 Kb8-a8 19. Ne5-c4 Bd6-e7 20. Bd2-f4 Qc7-c8 21. g3-g4 Nf6-d5 22. Bf4-d2 Be7-f6 23. Nc4-e5 Rh8-f8 24. c2-c4 Nd5-e7 25. Bd2-b4 Rd8-e8 26. f2-f4 Bf6xe5 27. f4xe5 f7-f5 28. Rh1-f1 Qc8-d7 29. Qe2-f2 Rf8-f7 30. Qf2-h4 b7-b6 31. Qh4-h1 Ka8-b7 32. Qh1-g2 g7-g5 33. h5xg6 Ne7xg6 34. g4xf5 Ng6-h4 35. f5xe6 Qd7xe6 36. Qg2-e4 Nh4-f5 37. Rf1-f3 a7-a5 38. Bb4-a3 h6-h5 39. b2-b3 Nf5-h6 40. Rf3xf7+ Nh6xf7 41. Ba3-b2 Nf7-g5 42. Qe4-d3 h5-h4 43. d4-d5 c6xd5 44. c4xd5 Qe6-g4 45. d5-d6 h4-h3 46. Rd1-c1 h3-h2 47. Qd3-d5+ Kb7-b8 48. d6-d7 Re8-d8 49. Qd5-d6+ 1-0",
    "1. e2-e4 d7-d6 2. d2-d4 Ng8-f6 3. Nb1-c3 g7-g6 4. Bc1-g5 c7-c6 5. f2-f4 Bf8-g7 6. Ng1-f3 b7-b5 7. Bf1-d3 O-O 8. O-O b5-b4 9. Nc3-e2 Bc8-b7 10. e4-e5 Nf6-d5 11. Qd1-d2 Qd8-b6 12. Kg1-h1 Nb8-d7 13. c2-c4 b4xc3 14. b2xc3 f7-f6 15. e5xf6 e7xf6 16. Bg5-h4 f6-f5 17. Ra1-b1 Qb6-c7 18. c3-c4 Nd5-f6 19. Ne2-c3 Rf8-e8 20. Qd2-c2 h7-h6 21. c4-c5 d6-d5 22. Nf3-e5 Nd7xe5 23. f4xe5 Nf6-d7 24. g2-g4 f5xg4 25. Bd3xg6 Nd7-f8 26. Rf1-f7 1-0",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Nb8-d7 5. Ne4-g5 Ng8-f6 6. Bf1-c4 e7-e6 7. Qd1-e2 Nd7-b6 8. Bc4-d3 h7-h6 9. Ng5-f3 c6-c5 10. d4xc5 Bf8xc5 11. Nf3-e5 Nb6-d7 12. Ng1-f3 Qd8-c7 13. O-O Nd7xe5 14. Nf3xe5 O-O 15. Bc1-f4 Bc5-d6 16. Ra1-d1 b7-b6 17. Bf4-g3 Bc8-b7 18. Ne5-g4 Nf6-e8 19. Bg3xd6 Ne8xd6 20. Qe2-e5 Ra8-c8 21. Rf1-e1 Nd6-f5 22. c2-c3 Nf5-h4 23. Bd3-e4 Bb7xe4 24. Re1xe4 Nh4-f5 25. Qe5xc7 Rc8xc7 26. Ng4-e5 Rf8-c8 27. Ne5-f3 b6-b5 28. a2-a4 b5xa4 29. Re4xa4 Rc8-b8 30. g2-g4 Nf5-e7 31. Rd1-d2 Ne7-d5 32. Nf3-d4 g7-g5 33. Kg1-g2 Nd5-f4+ 34. Kg2-g3 Rc7-d7 35. h2-h4 Rb8-d8 36. Ra4-a5 f7-f6 37. Rd2-e2 e6-e5 38. Nd4-f5 Rd7-d3+ 39. Nf5-e3 Rd8-d7 40. c3-c4 Rd7-c7 41. Ra5-b5 Kg8-f7 42. h4xg5 h6xg5 43. Rb5-b8 Kf7-g6 1/2-1/2",
    "1. c2-c4 e7-e5 2. e2-e3 Ng8-f6 3. Nb1-c3 Bf8-b4 4. Ng1-e2 O-O 5. a2-a3 Bb4xc3 6. Ne2xc3 Rf8-e8 7. Bf1-e2 e5-e4 8. d2-d3 e4xd3 9. Be2xd3 Nb8-c6 10. b2-b3 d7-d6 11. O-O Nc6-e5 12. Bd3-c2 Ne5-g6 13. Bc1-b2 c7-c6 14. Qd1-d4 c6-c5 15. Qd4-d2 Nf6-g4 16. Ra1-d1 Bc8-e6 17. Qd2xd6 Qd8-g5 18. Qd6-g3 Qg5-h5 19. Nc3-d5 Ra8-c8 20. h2-h3 Ng4-h6 21. Nd5-f4 Qh5-h4 22. Qg3xh4 Ng6xh4 23. Nf4xe6 Re8xe6 24. Rd1-d7 Re6-b6 25. Rf1-d1 Nh4-g6 26. Rd7-d8+ Rc8xd8 27. Rd1xd8+ Ng6-f8 28. Bb2-e5 1-0",
    "1. e2-e4 d7-d5 2. e4xd5 Ng8-f6 3. c2-c4 c7-c6 4. d2-d4 c6xd5 5. Nb1-c3 e7-e6 6. Ng1-f3 Bf8-b4 7. c4xd5 e6xd5 8. Bf1-d3 O-O 9. O-O Nb8-c6 10. Bc1-g5 Bb4xc3 11. b2xc3 Bc8-g4 12. Rf1-e1 Qd8-d6 13. h2-h3 Bg4-h5 14. Bd3-f5 Rf8-e8 15. Qd1-d3 Bh5-g6 16. Nf3-h4 Nc6-e7 17. Qd3-f3 Ne7xf5 18. Nh4xf5 Bg6xf5 19. Qf3xf5 Nf6-e4 20. Bg5-f4 Qd6-e6 21. Qf5-h5 Qe6-f6 22. Bf4-e5 Qf6xf2+ 23. Kg1-h2 g7-g6 24. Qh5-h6 Re8xe5 25. d4xe5 Qf2-g3+ 26. Kh2-g1 Qg3xe5 27. Ra1-c1 Ra8-c8 28. Qh6-e3 Rc8xc3 29. Rc1xc3 Qe5xc3 30. Qe3xc3 Ne4xc3 31. Re1-e7 Nc3xa2 32. Re7xb7 Na2-c3 33. Rb7xa7 Nc3-e2+ 34. Kg1-f2 Ne2-f4 35. Kf2-f3 Nf4-e6 36. Ra7-a5 Kg8-g7 37. Ra5xd5 Kg7-f6 38. Rd5-a5 h7-h5 39. h3-h4 Ne6-d4+ 40. Kf3-f4 Nd4-f5 41. Ra5-a6+ Kf6-g7 42. g2-g3 Nf5-h6 43. Kf4-e5 Nh6-f5 44. Ra6-a3 Nf5-h6 45. Ra3-f3 Nh6-f5 46. Rf3-b3 Nf5-h6 47. Ke5-d6 Nh6-f5+ 48. Kd6-d7 Kg7-f8 49. Rb3-a3 Nf5-h6 50. Ra3-a8+ Kf8-g7 51. Kd7-e8 Nh6-f5 52. Ra8-a3 Nf5-d6+ 53. Ke8-d7 Nd6-f5 54. Ra3-d3 Kg7-f8 55. Kd7-c6 Kf8-g7 56. Kc6-d5 Nf5-h6 57. Kd5-e5 Nh6-g4+ 58. Ke5-f4 1/2-1/2",
    "1. d2-d4 Ng8-f6 2. Bc1-g5 e7-e6 3. e2-e4 h7-h6 4. Bg5xf6 Qd8xf6 5. Ng1-f3 b7-b6 6. Bf1-d3 Bc8-b7 7. Nb1-d2 c7-c5 8. e4-e5 Qf6-d8 9. Bd3-e4 Qd8-c7 10. O-O Bf8-e7 11. d4xc5 b6xc5 12. Be4xb7 Qc7xb7 13. Nd2-c4 O-O 14. Qd1-d3 Nb8-c6 15. Ra1-d1 Ra8-d8 16. c2-c3 Qb7-a6 17. a2-a3 Qa6-b5 18. Rd1-d2 f7-f6 19. Rf1-d1 Qb5-b8 20. Qd3-g6 Qb8-c7 21. h2-h3 Rf8-f7 22. Qg6-g3 Be7-f8 23. Nf3-h2 Kg8-h8 24. Qg3-g6 Rf7-e7 25. e5xf6 g7xf6 26. Nh2-g4 Qc7-f4 27. Nc4-d6 Nc6-e5 28. Ng4xe5 Qf4xe5 29. Nd6-f7+ Re7xf7 30. Qg6xf7 d7-d5 31. Rd2-d3 Bf8-d6 32. Rd3-g3 f6-f5 33. Rg3-g6 1-0",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 Ng8-f6 4. e4-e5 Nf6-d7 5. f2-f4 c7-c5 6. c2-c3 Nb8-c6 7. Nd2-f3 Qd8-b6 8. g2-g3 c5xd4 9. c3xd4 Bf8-e7 10. Bf1-h3 h7-h5 11. Ng1-e2 g7-g6 12. Ra1-b1 a7-a5 13. Ke1-f1 Qb6-a6 14. Kf1-f2 Nd7-b6 15. Bh3-f1 a5-a4 16. Ne2-c3 Qa6-a5 17. Bc1-d2 Nb6-c4 18. Bd2-e1 Be7-b4 19. Bf1xc4 d5xc4 20. Nc3-e4 Ke8-e7 21. Ne4-d6 Qa5-d5 22. Be1xb4 Nc6xb4 23. Rh1-f1 Bc8-d7 24. Kf2-g1 Bd7-c6 25. Qd1-d2 Nb4-d3 26. b2-b3 a4xb3 27. a2xb3 b7-b5 28. b3xc4 b5xc4 29. f4-f5 e6xf5 30. Qd2-g5+ Ke7-d7 31. Qg5-f6 Rh8-f8 32. Rb1-b6 Nd3-e1 33. Rf1xe1 Qd5xf3 34. Rb6xc6 Ra8-a2 35. Rc6-c7+ Kd7xc7 36. Qf6-e7+ Kc7-b6 37. Nd6xc4+ Kb6-a6 38. Qe7-d6+ Ka6-a7 39. Qd6-c5+ Ka7-a8 40. Qc5xf8+ Ka8-a7 41. Qf8-c5+ Ka7-a8 42. Qc5-c8+ 1-0",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Nb1-c3 Bf8-b4 4. Ng1-f3 b7-b6 5. Bc1-g5 Bc8-b7 6. e2-e3 h7-h6 7. Bg5-h4 g7-g5 8. Bh4-g3 Nf6-e4 9. Qd1-c2 Bb4xc3+ 10. b2xc3 Ne4xg3 11. f2xg3 Nb8-c6 12. Bf1-d3 Qd8-e7 13. O-O O-O-O 14. a2-a4 h6-h5 15. a4-a5 Nc6xa5 16. Nf3-e5 Rh8-f8 17. Bd3-e4 d7-d5 18. c4xd5 e6xd5 19. Be4-f5+ Kc8-b8 20. e3-e4 Rd8-d6 21. Rf1-e1 Qe7-d8 22. Qc2-a4 Kb8-a8 23. e4xd5 Bb7xd5 24. c3-c4 Bd5-b7 25. Ne5xf7 Rf8xf7 26. Re1-e8 Rf7xf5 27. d4-d5 Rf5-f8 28. Ra1-e1 Rf8xe8 29. Re1xe8 Bb7-c8 30. Re8xd8 Rd6xd8 31. Qa4-c2 Ka8-b8 32. Qc2-e4 g5-g4 33. Qe4-e5 Na5xc4 34. Qe5xh5 a7-a5 35. h2-h4 a5-a4 36. Qh5-g5 Rd8-e8 37. Qg5-c1 Nc4-d6 38. h4-h5 Re8-h8 39. h5-h6 Rh8-h7 40. Kg1-f2 Bc8-d7 41. Qc1-b2 Rh7xh6 0-1",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 c7-c5 4. e4xd5 Qd8xd5 5. Ng1-f3 c5xd4 6. Bf1-c4 Qd5-d6 7. O-O Ng8-f6 8. Nd2-b3 Nb8-c6 9. Nb3xd4 Nc6xd4 10. Nf3xd4 a7-a6 11. c2-c3 Qd6-c7 12. Bc4-b3 Bf8-d6 13. h2-h3 O-O 14. Bc1-g5 b7-b5 15. Bb3-c2 Bc8-b7 16. Bg5xf6 g7xf6 17. Qd1-g4+ Kg8-h8 18. Qg4-h4 f6-f5 19. Bc2xf5 f7-f6 20. Nd4xe6 Qc7-d7 21. Ra1-d1 Rf8-f7 22. Ne6-g5 Bd6-h2+ 23. Kg1-h1 1-0",
    "1. g2-g3 Ng8-f6 2. Bf1-g2 e7-e5 3. d2-d3 Bf8-e7 4. c2-c4 O-O 5. Nb1-c3 c7-c6 6. e2-e4 d7-d6 7. Ng1-f3 Nb8-d7 8. O-O Qd8-c7 9. h2-h3 a7-a5 10. Bc1-e3 Nd7-c5 11. Ra1-c1 Nf6-d7 12. b2-b3 Nc5-e6 13. a2-a3 Qc7-d8 14. b3-b4 Rf8-e8 15. Kg1-h2 h7-h6 16. d3-d4 e5xd4 17. Nf3xd4 Nd7-e5 18. Nc3-e2 a5xb4 19. a3xb4 Ra8-a3 20. Nd4-b3 Ne6-f8 21. f2-f4 Ne5-g6 22. Qd1-c2 Nf8-d7 23. Rc1-d1 Qd8-c7 24. Be3-c1 Ra3-a8 25. Ne2-d4 Be7-f8 26. c4-c5 d6xc5 27. b4xc5 Nd7-f6 28. e4-e5 Nf6-d5 29. Bg2xd5 c6xd5 30. e5-e6 Ng6-h8 31. f4-f5 f7xe6 32. f5xe6 Bc8xe6 33. Nd4xe6 Re8xe6 34. Rd1xd5 Qc7-c6 35. Rd5-f5 Qc6-e4 36. Qc2-f2 Nh8-g6 37. Bc1-b2 Qe4-c4 38. Nb3-d4 Re6-e8 39. Rf1-c1 Qc4-d3 40. c5-c6 b7xc6 41. Rc1xc6 Ng6-e5 42. Rc6-e6 Ne5-c4 43. Rf5-f3 Qd3-b1 44. Rf3-b3 Re8xe6 45. Nd4xe6 Ra8-e8 46. Ne6xf8 Qb1-e1 47. Qf2xe1 Re8xe1 48. Bb2-d4 Kg8xf8 49. Rb3-b8+ Kf8-f7 50. Rb8-b7+ Re1-e7 51. Rb7xe7+ Kf7xe7 52. Bd4xg7 h6-h5 53. Bg7-d4 Ke7-e6 54. Kh2-g2 Ke6-f5 1/2-1/2",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 c7-c5 4. e4xd5 Qd8xd5 5. Ng1-f3 c5xd4 6. Bf1-c4 Qd5-d6 7. O-O Ng8-f6 8. Nd2-b3 Nb8-c6 9. Nb3xd4 Nc6xd4 10. Nf3xd4 Bc8-d7 11. Bc1-e3 Qd6-c7 12. Bc4-d3 O-O-O 13. Qd1-f3 e6-e5 14. Nd4-b5 e5-e4 15. Qf3-f4 Qc7xf4 16. Be3xf4 Bf8-c5 17. Bd3-c4 Bd7xb5 18. Bc4xb5 Nf6-d5 19. Bf4-g3 h7-h5 20. Bb5-e2 g7-g6 21. h2-h3 h5-h4 22. Bg3-h2 f7-f5 23. Be2-c4 e4-e3 24. f2xe3 Rh8-e8 25. Bc4-b5 Re8-e7 26. Kg1-h1 Nd5xe3 27. Rf1-e1 g6-g5 28. Bb5-d3 f5-f4 29. Bh2-g1 Rd8-f8 30. Bd3-e2 Kc8-c7 31. Be2-f3 Re7-d7 32. Re1-e2 Rf8-d8 33. Ra1-e1 Bc5-b6 34. Bg1xe3 f4xe3 35. g2-g3 h4xg3 36. Kh1-g2 Rd7-f7 37. Kg2xg3 Rd8-f8 38. Re1-f1 Bb6-c5 39. Kg3-g2 b7-b5 40. c2-c3 a7-a6 41. Re2-c2 Rf7-f4 42. Rc2-e2 Rf4-f6 43. Re2-e1 Rf8-d8 44. Re1-e2 Rd8-d2 45. Bf3-g4 Rf6-d6 46. Rf1-f7+ Kc7-d8 47. Rf7-f5 Rd6-d5 48. Kg2-f3 Rd5xf5+ 49. Bg4xf5 Rd2-d1 50. Bf5-g4 Kd8-e7 51. Kf3-e4 Rd1-f1 52. Ke4-d5 Bc5-b6 53. Kd5-c6 Rf1-f6+ 54. Kc6-b7 a6-a5 55. Kb7-a6 b5-b4 56. c3xb4 a5xb4 57. Ka6-b5 b4-b3 58. a2-a4 Bb6-d4 59. Kb5-c4 Rf6-f4 60. Kc4xb3 Ke7-d6 61. Kb3-c2 Kd6-c5 62. Kc2-d3 Kc5-b4 63. Re2-g2 Rf4-f2 1/2-1/2",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Nb8-c6 5. Nb1-c3 a7-a6 6. Bf1-e2 Qd8-c7 7. O-O Ng8-f6 8. Kg1-h1 b7-b5 9. Nd4xc6 d7xc6 10. f2-f4 b5-b4 11. e4-e5 b4xc3 12. e5xf6 g7xf6 13. b2xc3 f6-f5 14. Qd1-e1 Bc8-b7 15. Be2-h5 O-O-O 16. Ra1-b1 c6-c5 17. Qe1-e2 Rd8-d6 18. c3-c4 Bf8-g7 19. Bh5-f3 Bb7xf3 20. Qe2xf3 Rh8-d8 21. Bc1-e3 Rd6-c6 22. Be3-f2 Bg7-d4 23. Bf2-e1 Rd8-d6 24. Qf3-a3 Rd6-d7 25. Be1-a5 Qc7-a7 26. Qa3-f3 Rc6-d6 27. Rb1-b3 Rd7-b7 28. Rb3-d3 Rb7-b2 29. c2-c3 Rb2xa2 30. c3xd4 Ra2xa5 31. d4-d5 Qa7-b7 32. Rd3-b3 Rd6-b6 33. Rb3xb6 Qb7xb6 34. d5xe6 f7xe6 35. Rf1-d1 Qb6-b7 36. Qf3-e3 Qb7-e7 37. h2-h3 Ra5-a4 38. Qe3-f3 Kc8-b8 39. Qf3-c6 Ra4-b4 40. Rd1-d7 Rb4-b1+ 41. Kh1-h2 1-0",
    "1. d2-d4 Ng8-f6 2. Ng1-f3 e7-e6 3. Bc1-g5 b7-b6 4. c2-c4 Bc8-b7 5. Nb1-c3 Bf8-b4 6. e2-e3 h7-h6 7. Bg5-h4 g7-g5 8. Bh4-g3 Nf6-e4 9. Qd1-c2 Bb4xc3+ 10. b2xc3 Ne4xg3 11. f2xg3 Nb8-c6 12. Bf1-d3 Qd8-e7 13. O-O O-O-O 14. e3-e4 d7-d6 15. a2-a4 e6-e5 16. Qc2-f2 f7-f6 17. Nf3-d2 Rh8-f8 18. a4-a5 Nc6xa5 19. Rf1-b1 Kc8-b8 20. c4-c5 d6xc5 21. Ra1xa5 c5xd4 22. Nd2-b3 d4xc3 23. Nb3-c5 Rd8xd3 24. Nc5xd3 Bb7xe4 25. Qf2-e2 Qe7-h7 26. Ra5xa7 Be4xd3 27. Qe2-a2 Qh7-e4 28. Rb1-a1 c3-c2 29. Ra7-a8+ Kb8-b7 30. Ra8-a7+ Kb7-b8 31. Ra7-a8+ Kb8-b7 32. Ra8-a7+ Kb7-b8 1/2-1/2",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-c3 Bf8-b4 4. Bf1-d3 d5xe4 5. Bd3xe4 Ng8-f6 6. Be4-d3 c7-c5 7. a2-a3 Bb4-a5 8. d4xc5 Ba5xc3+ 9. b2xc3 Nb8-d7 10. Bc1-e3 Qd8-c7 11. Ng1-f3 O-O 12. O-O e6-e5 13. Nf3-d2 Nd7xc5 14. Be3-g5 e5-e4 15. Bg5xf6 e4xd3 16. Bf6-d4 d3xc2 17. Qd1xc2 Nc5-e6 18. Bd4-e3 b7-b6 19. f2-f4 Bc8-a6 20. Rf1-e1 Ra8-d8 21. Qc2-a4 Ne6-c5 22. Qa4-c2 Nc5-d3 23. Re1-f1 Rf8-e8 24. Rf1-f3 Ba6-b7 25. Rf3-h3 g7-g6 26. Ra1-f1 Re8xe3 27. Rh3xe3 Qc7-c5 28. Nd2-c4 b6-b5 29. Kg1-h1 b5xc4 30. Qc2-e2 Bb7-c6 31. f4-f5 Rd8-b8 32. Re3-g3 Rb8-e8 33. Qe2-d2 Qc5-e5 34. f5xg6 h7xg6 35. Qd2-h6 Qe5-f5 36. Rf1-a1 Nd3-e1 37. Kh1-g1 Re8-e2 38. Ra1-d1 Qf5-f2+ 0-1",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 g7-g6 6. Bf1-e2 Bf8-g7 7. O-O O-O 8. Bc1-g5 Nb8-c6 9. Nd4-b3 Bc8-e6 10. Kg1-h1 h7-h6 11. Bg5-h4 g6-g5 12. Bh4-g3 d6-d5 13. e4-e5 Nf6-e8 14. Nb3-d4 Nc6xe5 15. Nd4xe6 f7xe6 16. f2-f4 g5xf4 17. Rf1xf4 Rf8xf4 18. Bg3xf4 Ne5-g6 19. Bf4-g3 Qd8-b6 20. Ra1-b1 Qb6-e3 21. Be2-d3 Ng6-e5 22. Qd1-f1 Ne5xd3 23. c2xd3 Ne8-d6 24. Rb1-e1 Qe3-d2 25. Qf1-e2 Qd2xe2 26. Re1xe2 Ra8-f8 27. Re2-e1 Rf8-f5 28. h2-h3 Kg8-f7 29. Nc3-d1 Bg7-d4 30. Bg3-h2 e6-e5 31. Bh2-g1 h6-h5 32. Nd1-c3 Bd4xc3 33. b2xc3 e5-e4 34. Bg1-c5 e4xd3 35. Bc5xd6 e7xd6 36. Re1-d1 d5-d4 37. Rd1xd3 d4xc3 38. Rd3xc3 Kf7-e6 39. g2-g4 h5xg4 40. h3xg4 Rf5-g5 41. Rc3-c4 b7-b5 42. Rc4-f4 Ke6-e5 43. Rf4-b4 d6-d5 44. Kh1-g2 a7-a5 45. Rb4xb5 Rg5xg4+ 46. Kg2-f3 Rg4-a4 47. Rb5-b2 Ra4-a3+ 48. Kf3-f2 Ke5-d4 49. Kf2-e1 Ra3-h3 50. Rb2-g2 Kd4-c3 51. Rg2-g5 Rh3-h1+ 52. Ke1-e2 Rh1-h2+ 53. Ke2-d1 d5-d4 0-1",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. e4-e5 c7-c5 4. c2-c3 Nb8-c6 5. Ng1-f3 Bc8-d7 6. Bf1-e2 Ng8-e7 7. Nb1-a3 c5xd4 8. c3xd4 Ne7-f5 9. Na3-c2 Bf8-e7 10. O-O O-O 11. Be2-d3 f7-f6 12. Bd3xf5 e6xf5 13. Bc1-f4 Bd7-e8 14. Qd1-d2 Be8-h5 15. e5xf6 Be7xf6 16. Nf3-e5 Nc6-a5 17. Qd2-d3 Ra8-c8 18. Ra1-c1 Na5-c4 19. b2-b3 Nc4-d6 20. Nc2-b4 Qd8-a5 21. Ne5-c6 b7xc6 22. Bf4xd6 Rf8-e8 23. Qd3xf5 Bh5-g6 24. Qf5-d7 Re8-d8 25. Qd7-g4 Qa5-b5 26. Bd6-c5 Rd8-e8 27. Qg4-d7 Bg6-f7 28. a2-a4 Qb5-b8 29. Nb4xc6 Qb8xb3 30. Nc6xa7 Rc8-b8 31. Na7-c6 Rb8-b7 32. Nc6-a5 Rb7xd7 33. Na5xb3 Re8-b8 34. Rc1-c3 Rd7-b7 35. Nb3-a5 Rb7-b2 36. Na5-c6 Rb8-a8 37. a4-a5 Bf7-e8 38. Bc5-b6 Be8xc6 39. Rc3xc6 Rb2-d2 40. Rf1-a1 Bf6xd4 41. Bb6xd4 Rd2xd4 42. a5-a6 Rd4-b4 43. Kg1-f1 d5-d4 44. Kf1-e1 Rb4-b2 45. a6-a7 d4-d3 46. Rc6-d6 Rb2-e2+ 47. Ke1-f1 Re2-e7 48. Rd6xd3 Re7xa7 49. Ra1xa7 Ra8xa7 50. Kf1-e2 Ra7-a2+ 51. Rd3-d2 Ra2-a3 52. f2-f4 g7-g6 53. Rd2-d3 Ra3-a2+ 54. Ke2-f3 h7-h5 55. h2-h4 Kg8-f7 56. Rd3-b3 1/2-1/2",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 Ng8-f6 4. e4-e5 Nf6-d7 5. c2-c3 c7-c5 6. f2-f4 Nb8-c6 7. Nd2-f3 Qd8-b6 8. g2-g3 Bf8-e7 9. Bf1-h3 c5xd4 10. c3xd4 f7-f6 11. Ke1-f1 O-O 12. Kf1-g2 f6xe5 13. f4xe5 Nd7xe5 14. d4xe5 Nc6xe5 15. Qd1-e2 Ne5-c4 16. b2-b3 Be7-f6 17. Ra1-b1 Kg8-h8 18. Qe2-f2 d5-d4 19. Qf2-c2 1/2-1/2",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 Bf8-b4 6. Bf1-d3 e6-e5 7. Nd4-e2 d7-d5 8. e4xd5 Nf6xd5 9. Bc1-d2 Nb8-c6 10. O-O Nd5xc3 11. Ne2xc3 O-O 12. Qd1-h5 g7-g6 13. Qh5-h6 Bc8-e6 14. Ra1-d1 f7-f5 15. a2-a3 Bb4-e7 16. f2-f3 Be7-f6 17. Kg1-h1 Qd8-e7 18. Qh6-e3 Ra8-c8 19. Nc3-b5 b7-b6 20. Qe3-g1 Rf8-d8 21. Nb5-c3 Qe7-b7 22. Rf1-e1 Nc6-a5 23. Bd2-c1 Na5-c4 24. Qg1-f1 Qb7-f7 25. Nc3-b1 Nc4-d6 26. Nb1-d2 Nd6-b7 27. b2-b3 Nb7-d6 28. Bc1-b2 Qf7-g7 1/2-1/2",
    "1. e2-e4 g7-g6 2. d2-d4 Bf8-g7 3. Nb1-c3 c7-c6 4. Bf1-c4 b7-b5 5. Bc4-b3 b5-b4 6. Nc3-e2 Ng8-f6 7. e4-e5 Nf6-d5 8. Ne2-f4 Nd5xf4 9. Bc1xf4 O-O 10. h2-h4 d7-d6 11. Qd1-e2 a7-a5 12. h4-h5 d6-d5 13. Bb3-a4 c6-c5 14. d4xc5 Bc8-d7 15. Ba4xd7 Nb8xd7 16. c5-c6 Nd7-c5 17. Ng1-f3 Nc5-e6 18. Bf4-e3 Qd8-c7 19. Qe2-b5 Bg7xe5 20. O-O-O Be5-f6 21. h5xg6 h7xg6 22. Rd1xd5 Ra8-b8 23. Qb5-a4 Rb8-a8 24. Rd5-d7 Qc7-c8 25. Be3-b6 Qc8-a6 26. Rd7-b7 Rf8-c8 27. Rh1-d1 Qa6-c4 28. c6-c7 Ne6xc7 29. Bb6-d4 Nc7-e6 30. Qa4-b3 Qc4-c6 31. Bd4xf6 Qc6xb7 32. Bf6-e5 Ne6-c5 33. Qb3-c4 Nc5-e4 34. Rd1-h1 Ne4-f6 35. Be5xf6 e7xf6 36. Qc4-h4 Kg8-f8 37. Qh4xf6 Rc8xc2+ 38. Kc1-b1 1-0",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-c3 d5xe4 4. Nc3xe4 Nb8-d7 5. Ng1-e2 Ng8-f6 6. Ne2-g3 g7-g6 7. Bf1-c4 Bf8-g7 8. c2-c3 O-O 9. O-O Nf6-d5 10. Rf1-e1 h7-h6 11. Bc4-f1 Nd7-f6 12. Bc1-d2 b7-b6 13. Qd1-b3 Qd8-d7 14. a2-a4 Qd7-c7 15. a4-a5 Nf6xe4 16. Ng3xe4 Bc8-f5 17. Qb3-a3 b6-b5 18. Ne4-c5 Rf8-e8 19. Bf1-d3 Bf5xd3 20. Nc5xd3 e7-e6 21. Ra1-c1 Ra8-b8 22. b2-b3 Qc7-e7 23. Qa3-a2 Re8-d8 24. c3-c4 b5xc4 25. b3xc4 Nd5-c7 26. Nd3-e5 Qe7-e8 27. Bd2-c3 Nc7-a6 28. Qa2-a4 c6-c5 29. Qa4xe8+ Rd8xe8 30. Ne5-c6 Rb8-b3 31. Bc3-a1 Na6-b4 32. Nc6xb4 Rb3xb4 33. d4xc5 Bg7xa1 34. Rc1xa1 Re8-c8 35. Ra1-d1 Rc8xc5 36. Rd1-d7 Rc5xa5 37. Rd7-c7 Rb4-b2 38. g2-g3 Rb2-c2 39. Re1-d1 Ra5-f5 40. Rd1-f1 a7-a5 41. h2-h4 a5-a4 42. Rc7-a7 Rc2xc4 43. Kg1-g2 Kg8-g7 44. Kg2-g1 g6-g5 45. h4xg5 h6xg5 46. Kg1-g2 g5-g4 47. Kg2-g1 Rf5-c5 48. Kg1-g2 Rc5-d5 49. Rf1-a1 Rd5-d4 50. Ra1-a2 Kg7-g6 51. Ra2-a1 f7-f5 52. Ra7-a6 Rd4-e4 53. Ra6-a5 Kg6-f6 54. Ra5-a6 f5-f4 55. g3xf4 Re4xf4 56. Ra1-a2 Rf4-e4 57. Ra6-a5 Kf6-g6 58. Ra2-a1 Kg6-f6 59. Ra1-a2 Kf6-e7 60. Ra5-a6 Ke7-d7 61. Ra6-a7+ Kd7-c6 62. Ra7-a8 Rc4-b4 63. Ra2-a3 e6-e5 64. Ra8-a6+ Kc6-b5 65. Ra6-a8 Re4-f4 66. Ra8-e8 Rb4-e4 67. Re8-b8+ Kb5-c5 68. Rb8-a8 Re4-e2 69. Ra8xa4 Re2xf2+ 70. Kg2-g3 Rf2-f3+ 71. Ra3xf3 Rf4xa4 72. Rf3-f8 Ra4-d4 73. Rf8-a8 Kc5-d5 74. Ra8-a3 Rd4-f4 75. Ra3-a5+ Kd5-e4 76. Ra5-a8 Rf4-f1 77. Ra8-a7 Rf1-g1+ 78. Kg3-f2 Rg1-d1 79. Kf2-g3 Rd1-f1 80. Ra7-a8 Rf1-f4 81. Ra8-a7 Ke4-d5 82. Ra7-a5+ Kd5-e6 83. Ra5-a8 Rf4-d4 84. Ra8-e8+ Ke6-d5 85. Re8-a8 Kd5-c4 86. Ra8-a5 Rd4-e4 87. Ra5-a4+ Kc4-d5 88. Ra4-a8 Re4-b4 89. Ra8-a5+ Kd5-e6 90. Ra5-a8 Ke6-f5 91. Ra8-f8+ Kf5-e4 92. Kg3xg4 Ke4-d3+ 93. Kg4-g3 e5-e4 94. Rf8-a8 Rb4-c4 95. Ra8-a3+ Rc4-c3 96. Ra3-a2 Kd3-e3 97. Ra2-a1 Ke3-d4+ 98. Kg3-f2 Rc3-c2+ 99. Kf2-e1 Rc2-b2 1/2-1/2",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4xd5 c6xd5 4. c2-c4 Ng8-f6 5. Nb1-c3 e7-e6 6. Ng1-f3 Bf8-e7 7. c4xd5 Nf6xd5 8. Bf1-d3 Nb8-c6 9. O-O O-O 10. a2-a3 Nd5-f6 11. Bd3-c2 b7-b6 12. Qd1-d3 a7-a5 13. Nc3-e4 g7-g6 14. Rf1-d1 Bc8-a6 15. Qd3-e3 Rf8-e8 16. Nf3-e5 Nf6-d5 17. Qe3-h6 Qd8-c7 18. Ne5xc6 Qc7xc6 19. Bc2-b3 Ba6-c4 20. Bb3xc4 Qc6xc4 21. Bc1-g5 Be7-f8 22. Qh6-h4 Ra8-c8 23. Bg5-f6 Qc4-c7 24. Bf6-e5 Qc7-d8 25. Qh4-g3 f7-f6 26. Be5-d6 Rc8-c6 27. Bd6xf8 Re8xf8 28. Ra1-c1 Qd8-d7 29. Rc1xc6 Qd7xc6 30. Rd1-e1 Kg8-g7 31. h2-h4 Rf8-d8 32. Qg3-f3 Qc6-c4 33. Ne4-c3 Kg7-f7 34. Qf3-e4 Nd5-e7 35. Qe4-b7 Qc4-c6 36. Qb7xc6 Ne7xc6 37. Nc3-a4 b6-b5 38. Na4-c5 Nc6xd4 39. Nc5-b7 Rd8-a8 40. Re1-d1 e6-e5 41. f2-f4 Kf7-e6 42. f4xe5 f6xe5 43. b2-b4 a5xb4 44. a3xb4 Ke6-d5 45. Rd1-f1 e5-e4 46. Rf1-f7 e4-e3 47. Rf7-d7+ Kd5-c4 48. Nb7-a5+ Kc4-c3 0-1",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 O-O 8. a2-a4 Bc8-b7 9. d2-d3 d7-d6 10. Nb1-c3 Nc6-a5 11. Bb3-a2 b5-b4 12. Nc3-e2 c7-c5 13. Ne2-g3 Bb7-c8 14. Nf3-d2 Bc8-e6 15. Nd2-c4 Na5xc4 16. Ba2xc4 Be6xc4 17. d3xc4 Ra8-b8 18. Qd1-d3 Nf6-e8 19. Ng3-f5 Be7-f6 20. Re1-d1 Rb8-b6 21. Qd3-g3 Kg8-h8 22. Rd1-d3 Ne8-c7 23. Nf5-e3 Nc7-e6 24. Ne3-d5 Rb6-c6 25. Qg3-g4 Bf6-g5 26. Bc1xg5 Ne6xg5 27. Ra1-d1 Ng5-e6 28. Rd3-g3 g7-g6 29. Rg3-f3 Qd8-g5 30. Qg4xg5 Ne6xg5 31. Rf3-e3 a6-a5 32. h2-h4 Ng5-e6 33. g2-g3 Kh8-g7 34. f2-f4 f7-f6 35. Kg1-g2 Rf8-b8 36. Rd1-d2 Rb8-b7 37. Rd2-f2 Rb7-f7 38. c2-c3 Ne6-d8 39. Re3-f3 Rc6-a6 40. f4-f5 g6xf5 41. Rf3xf5 Nd8-e6 42. Kg2-h2 Ne6-c7 43. Nd5xf6 Kg7-h8 44. Nf6xh7 Rf7xf5 45. Rf2xf5 Kh8xh7 46. Rf5-f7+ Kh7-g6 47. Rf7xc7 b4xc3 48. b2xc3 Ra6-b6 49. Rc7-d7 Kg6-f6 50. Kh2-h3 Kf6-e6 51. Rd7-d8 Ke6-e7 52. Rd8-a8 Rb6-b3 53. h4-h5 Ke7-f6 54. Ra8-f8+ Kf6-g7 55. Rf8-f3 Rb3-a3 56. Kh3-h4 Kg7-h6 57. g3-g4 Ra3-a1 58. Rf3-f6+ Kh6-h7 59. Kh4-g5 1-0",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 Ng8-f6 4. e4-e5 Nf6-d7 5. c2-c3 c7-c5 6. f2-f4 c5xd4 7. c3xd4 Nb8-c6 8. Nd2-f3 h7-h5 9. Bf1-d3 Nd7-b6 10. Ng1-e2 Bc8-d7 11. O-O Bf8-e7 12. Ne2-c3 g7-g6 13. Qd1-e2 Ke8-f8 14. Bc1-e3 Kf8-g7 15. Be3-f2 a7-a5 16. Ra1-d1 a5-a4 17. a2-a3 Nc6-a5 18. Nf3-d2 Ra8-c8 19. Rd1-c1 Nb6-c4 20. Nd2xc4 Na5xc4 21. Rc1-c2 Nc4xb2 22. Rc2xb2 Rc8xc3 23. Rb2xb7 Bd7-c6 24. Rb7-a7 Be7xa3 25. Bf2-e1 Rc3-c1 26. Bd3xg6 Kg7xg6 27. Qe2-d3+ Kg6-g7 28. Qd3-g3+ Kg7-h6 29. Qg3xa3 Rc1-b1 30. Ra7xf7 Bc6-b5 31. f4-f5 e6xf5 32. Be1-d2+ Kh6-g6 33. Rf1xb1 Kg6xf7 34. Rb1xb5 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 g7-g6 4. d2-d4 e5xd4 5. Nf3xd4 Bf8-g7 6. Bc1-e3 Ng8-e7 7. Nb1-c3 O-O 8. O-O d7-d6 9. Qd1-d2 a7-a6 10. Bb5-e2 Nc6xd4 11. Be3xd4 Bg7xd4 12. Qd2xd4 Bc8-e6 13. Ra1-d1 f7-f6 14. f2-f4 Qd8-e8 15. Be2-f3 Ra8-d8 16. Rf1-e1 c7-c5 17. Qd4-f2 b7-b6 18. Rd1-d2 Ne7-c6 19. g2-g3 Nc6-d4 20. Bf3-g2 Qe8-f7 21. Nc3-d5 Be6xd5 22. e4xd5 Rf8-e8 23. c2-c3 Re8xe1+ 24. Qf2xe1 Rd8-e8 25. Qe1-f2 Nd4-b5 26. Bg2-f3 Nb5-c7 27. b2-b4 Nc7-b5 28. Rd2-c2 a6-a5 29. b4xa5 b6xa5 30. a2-a4 Nb5-c7 31. Rc2-b2 Nc7xd5 32. Qf2-d2 Nd5-e3 33. Qd2xd6 Qf7-c4 34. Rb2-e2 Qc4xc3 35. Kg1-f2 Qc3-d4 36. Qd6xd4 c5xd4 37. Re2-b2 Re8-c8 38. Bf3-e2 Ne3-d5 39. Rb2-d2 Nd5-c3 40. Rd2xd4 Nc3xe2 41. Kf2xe2 Rc8-c2+ 42. Ke2-f3 Rc2-a2 1/2-1/2",
    "1. e2-e4 c7-c5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 g7-g6 4. O-O Bf8-g7 5. Rf1-e1 a7-a6 6. Bb5xc6 d7xc6 7. Nb1-c3 e7-e5 8. a2-a4 a6-a5 9. b2-b3 Ng8-e7 10. Bc1-b2 O-O 11. d2-d3 Qd8-c7 12. Kg1-h1 Bc8-d7 13. Qd1-d2 h7-h6 14. Nc3-e2 b7-b6 15. Nf3-g1 Ra8-e8 16. f2-f4 e5xf4 17. Bb2xg7 Kg8xg7 18. Ne2xf4 Ne7-g8 19. Ng1-f3 Bd7-g4 20. Re1-f1 g6-g5 21. Nf4-e2 f7-f5 22. e4xf5 Bg4xf5 23. Ne2-g3 Bf5-g6 24. Ra1-e1 Ng8-f6 25. Re1xe8 Rf8xe8 26. Qd2-c3 Qc7-d6 27. Qc3-a1 Re8-e6 28. d3-d4 Kg7-h7 29. d4xc5 b6xc5 30. Qa1-c3 Nf6-d5 31. Qc3xa5 Nd5-e3 32. Rf1-e1 g5-g4 33. Nf3-d2 Bg6xc2 34. Qa5-c3 Qd6-d4 35. Re1xe3 Re6xe3 36. Qc3xc2+ Re3-d3 37. Ng3-e2 Qd4-e3 38. Nd2-c4 Qe3-e4 39. Kh1-g1 Kh7-h8 40. a4-a5 Qe4-d5 41. Qc2-b2+ Kh8-g8 42. Ne2-f4 Rd3-d1+ 43. Kg1-f2 Qd5-e4 44. Qb2-e5 Rd1-d2+ 45. Kf2-g3 Rd2xg2+ 46. Kg3-h4 Rg2xh2+ 47. Kh4xg4 Rh2-g2+ 48. Kg4-h4 Rg2-h2+ 49. Kh4-g3 Qe4xe5 50. Nc4xe5 Rh2-b2 51. Ne5xc6 Rb2xb3+ 52. Kg3-h4 Rb3-a3 53. Nf4-d5 Kg8-f7 54. Nd5-c7 Ra3-a4+ 55. Kh4-h5 c5-c4 56. Nc7-b5 Kf7-g7 57. Nb5-c3 Ra4-a1 58. Kh5-g4 Kg7-h7 59. Nc3-e4 c4-c3 60. Ne4xc3 Ra1-c1 61. a5-a6 Rc1xc3 62. a6-a7 Rc3-a3 63. Kg4-h4 Ra3-a4+ 64. Kh4-h3 Kh7-g7 65. Kh3-g3 Kg7-f6 66. Kg3-h3 Kf6-g5 67. Kh3-g3 Ra4-a3+ 68. Kg3-g2 h6-h5 69. Kg2-h2 h5-h4 70. Nc6-e5 Ra3xa7 71. Ne5-f3+ Kg5-f4 72. Nf3-d4 Kf4-g4 73. Kh2-g2 Ra7-a2+ 74. Kg2-g1 Ra2-d2 75. Nd4-b3 Rd2-d5 76. Kg1-h2 h4-h3 77. Nb3-c1 Rd5-d2+ 78. Kh2-g1 Kg4-g3 0-1",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Nb8-d7 5. Bf1-c4 Ng8-f6 6. Ne4-g5 e7-e6 7. Qd1-e2 Nd7-b6 8. Bc4-b3 a7-a5 9. a2-a3 a5-a4 10. Bb3-a2 h7-h6 11. Ng5-f3 c6-c5 12. Bc1-f4 Nb6-d5 13. Bf4-e5 Qd8-a5+ 14. Qe2-d2 Qa5xd2+ 15. Nf3xd2 Nf6-g4 16. Ng1-f3 Ng4xe5 17. d4xe5 b7-b6 18. c2-c4 Nd5-e7 19. O-O-O Bc8-b7 20. Ba2-b1 Ne7-c6 21. Bb1-c2 Bf8-e7 22. Nd2-e4 Nc6-b8 23. Ne4-d6+ Be7xd6 24. Rd1xd6 Nb8-d7 25. Rh1-d1 Bb7xf3 26. g2xf3 Nd7xe5 27. Bc2xa4+ Ke8-e7 28. Ba4-b5 Ra8-b8 29. f3-f4 Ne5-f3 30. Rd6-d7+ Ke7-f6 31. f4-f5 Nf3xh2 32. f5xe6 f7xe6 33. Bb5-c6 Nh2-g4 34. f2-f4 Ng4-e3 35. Rd1-g1 g7-g5 36. f4xg5+ h6xg5 37. Rg1-e1 Ne3-f5 38. Bc6-e4 Rb8-d8 39. Rd7-b7 Rd8-d6 40. b2-b4 Rh8-h3 41. b4xc5 b6xc5 42. Rb7-c7 Rh3xa3 43. Rc7xc5 g5-g4 44. Kc1-b2 Ra3-h3 45. Re1-f1 Rh3-h5 46. Rc5-c8 Rd6-d2+ 47. Kb2-c3 Rd2-e2 48. Rc8-f8+ Kf6-e7 49. Be4xf5 e6xf5 1/2-1/2",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-c3 Bf8-b4 4. e4-e5 c7-c5 5. a2-a3 Bb4xc3+ 6. b2xc3 Ng8-e7 7. Ng1-f3 Qd8-a5 8. Bc1-d2 Nb8-c6 9. Bf1-e2 c5xd4 10. c3xd4 Qa5-a4 11. c2-c4 1/2-1/2",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-c3 d5xe4 4. Nc3xe4 Nb8-d7 5. Ng1-f3 Ng8-f6 6. Ne4xf6+ Nd7xf6 7. Bf1-c4 Bc8-f5 8. Qd1-e2 e7-e6 9. Bc1-g5 Qd8-a5+ 10. Bg5-d2 Bf8-b4 11. Bd2xb4 Qa5xb4+ 12. c2-c3 Qb4-e7 13. Nf3-e5 h7-h5 14. Bc4-d3 g7-g6 15. Ne5-c4 h5-h4 16. h2-h3 Nf6-d5 17. Bd3xf5 g6xf5 18. Nc4-e3 O-O-O 19. O-O-O Qe7-g5 20. Kc1-b1 Rh8-g8 21. Rh1-g1 Qg5-f4 22. Ne3-c4 Rg8-g7 23. Nc4-e5 Qf4-e4+ 24. Qe2xe4 f5xe4 25. Rd1-e1 f7-f5 26. f2-f3 e4-e3 27. Ne5-d3 b7-b5 28. Kb1-c2 f5-f4 29. Nd3-e5 Kc8-c7 30. Ne5-g4 Nd5-e7 31. b2-b4 a7-a5 32. a2-a3 Rd8-a8 33. Re1-a1 Ne7-d5 34. Kc2-b2 Kc7-d6 35. Kb2-c2 Rg7-a7 36. b4xa5 Ra7xa5 37. Kc2-b3 Nd5-b6 38. Ng4-e5 Ra5-a4 39. Ne5-g6 Nb6-c4 40. Ng6xf4 Ra4xa3+ 41. Ra1xa3 Ra8xa3+ 42. Kb3-c2 Ra3-a2+ 43. Kc2-d3 e6-e5 44. d4xe5+ Kd6xe5 45. Nf4-g6+ Ke5-d5 46. Ng6-f4+ Kd5-d6 47. Rg1-e1 c6-c5 48. Re1-e2 Ra2-d2+ 49. Re2xd2 e3xd2 50. Kd3-c2 b5-b4 51. c3xb4 c5xb4 52. Nf4-d3 b4-b3+ 53. Kc2-d1 b3-b2 54. Nd3xb2 Nc4xb2+ 55. Kd1xd2 Kd6-e5 56. Kd2-e3 Nb2-c4+ 57. Ke3-d3 Nc4-d6 58. Kd3-e3 Nd6-f5+ 59. Ke3-f2 Ke5-f4 60. Kf2-g1 Kf4-g3 61. Kg1-h1 Nf5-e3 62. f3-f4 Kg3xf4 63. Kh1-h2 Ne3-f5 64. Kh2-g1 Kf4-g3 65. Kg1-h1 Nf5-e3 66. Kh1-g1 Ne3xg2 67. Kg1-h1 Ng2-f4 68. Kh1-g1 Nf4xh3+ 69. Kg1-f1 Nh3-f2 0-1",
    "1. c2-c4 e7-e5 2. Nb1-c3 Ng8-f6 3. Ng1-f3 Nb8-c6 4. e2-e3 Bf8-b4 5. Qd1-c2 O-O 6. Nc3-d5 Rf8-e8 7. Qc2-f5 d7-d6 8. Nd5xf6+ g7xf6 9. Qf5-h5 d6-d5 10. Bf1-d3 e5-e4 11. c4xd5 e4xd3 12. d5xc6 b7xc6 13. b2-b3 Bb4-f8 14. Bc1-b2 Re8-e4 15. h2-h3 Ra8-b8 16. Nf3-d4 Qd8-d5 17. Qh5xd5 c6xd5 18. Nd4-f3 Re4-e6 19. O-O Bc8-a6 20. Rf1-c1 Rb8-b7 21. Nf3-d4 Re6-b6 22. Nd4-f3 Ba6-b5 23. Bb2-d4 Bf8-a3 24. Bd4xb6 a7xb6 25. Nf3-d4 Ba3xc1 26. Ra1xc1 Bb5-d7 27. Rc1-c3 c7-c5 28. Nd4-f3 Bd7-b5 29. a2-a4 Bb5-a6 30. b3-b4 Ba6-c4 31. b4xc5 b6xc5 32. Kg1-h2 Rb7-b2 33. Kh2-g3 Rb2-c2 34. Kg3-f4 Kg8-f8 35. g2-g4 h7-h6 36. h3-h4 Kf8-e7 37. Kf4-f5 Bc4-a6 38. g4-g5 f6xg5 39. h4xg5 h6-h5 40. Rc3-b3 c5-c4 41. Rb3-a3 d5-d4 42. e3xd4 0-1",
    "1. c2-c4 e7-e5 2. Nb1-c3 Ng8-f6 3. Ng1-f3 Nb8-c6 4. e2-e4 Bf8-b4 5. d2-d3 O-O 6. g2-g3 d7-d6 7. Bf1-g2 Bc8-g4 8. h2-h3 Bg4xf3 9. Bg2xf3 Nc6-d4 10. Bf3-g2 a7-a5 11. O-O c7-c6 12. Bc1-e3 Bb4-c5 13. Kg1-h2 Nd4-e6 14. Qd1-d2 Qd8-e7 15. Nc3-e2 Bc5xe3 16. Qd2xe3 c6-c5 17. Qe3-d2 Nf6-e8 18. Ra1-e1 Ne8-c7 19. Ne2-c3 a5-a4 20. f2-f4 Ne6-d4 21. Qd2-d1 Qe7-d7 22. f4xe5 d6xe5 23. Nc3-d5 Ra8-a6 24. Rf1-f2 b7-b5 25. Qd1-h5 f7-f6 26. Re1-f1 Rf8-b8 27. h3-h4 b5xc4 28. d3xc4 Nc7xd5 29. e4xd5 g7-g6 30. Qh5-h6 f6-f5 31. d5-d6 Ra6xd6 32. Bg2-d5+ Kg8-h8 33. Rf1-e1 Qd7-e7 34. Qh6-e3 e5-e4 35. Qe3-h6 Qe7-g7 36. Qh6-g5 Rb8xb2 37. Rf2xb2 Nd4-f3+ 38. Kh2-h1 Qg7xb2 0-1",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4xd5 c6xd5 4. c2-c4 Ng8-f6 5. Nb1-c3 e7-e6 6. Ng1-f3 Bf8-e7 7. c4xd5 Nf6xd5 8. Bf1-d3 Nb8-c6 9. O-O O-O 10. a2-a3 Nd5-f6 11. Bd3-c2 b7-b6 12. Rf1-e1 Bc8-a6 13. b2-b4 Ra8-c8 14. Bc1-b2 Nf6-d5 15. b4-b5 Nd5xc3 16. Qd1-d3 g7-g6 17. Bb2xc3 Qd8-c7 18. b5xa6 Nc6-a5 19. Bc3xa5 1-0",
    "1. d2-d4 Ng8-f6 2. Ng1-f3 e7-e6 3. Bc1-g5 h7-h6 4. Bg5xf6 Qd8xf6 5. e2-e4 d7-d6 6. Nb1-c3 c7-c6 7. e4-e5 Qf6-d8 8. e5xd6 Bf8xd6 9. Nc3-e4 Bd6-e7 10. Bf1-d3 Nb8-d7 11. O-O Qd8-c7 12. c2-c4 b7-b6 13. Ra1-c1 O-O 14. d4-d5 Nd7-c5 15. Ne4xc5 b6xc5 16. d5xc6 Qc7xc6 17. Nf3-e5 Qc6-c7 18. Qd1-e2 Bc8-b7 19. Rc1-d1 Ra8-d8 20. Bd3-e4 Be7-f6 21. f2-f4 g7-g6 22. Be4xb7 Qc7xb7 23. b2-b3 a7-a5 24. h2-h3 Kg8-g7 25. Kg1-h1 h6-h5 26. Rf1-f3 Rd8xd1+ 27. Qe2xd1 Qb7-c7 28. Qd1-e1 Rf8-d8 29. Rf3-d3 Rd8xd3 30. Ne5xd3 Bf6-d4 31. Qe1-e4 h5-h4 32. Qe4-e1 Bd4-f6 33. Qe1-e3 Qc7-d6 34. Nd3-e5 g6-g5 35. Ne5-d3 g5xf4 36. Nd3xf4 Qd6-d4 37. Qe3xd4 Bf6xd4 38. g2-g4 h4xg3 39. Nf4-h5+ Kg7-g6 40. Nh5xg3 f7-f5 41. Kh1-g2 Kg6-g5 42. Kg2-f3 Bd4-e5 43. Ng3-e2 Kg5-h4 0-1",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Bf8-c5 5. Nd4-b3 Bc5-b6 6. Nb1-c3 Ng8-e7 7. Bc1-f4 O-O 8. Bf4-d6 Nb8-c6 9. Bf1-e2 f7-f5 10. O-O f5-f4 11. Qd1-d2 Bb6-c7 12. Nc3-b5 Bc7xd6 13. Nb5xd6 Ne7-g6 14. f2-f3 Nc6-e5 15. Kg1-h1 Qd8-g5 16. Rf1-f2 Rf8-f6 17. Qd2-c3 Ne5-c6 18. Ra1-d1 Ng6-e5 19. Be2-f1 Rf6-h6 20. Qc3-e1 b7-b6 21. Rf2-d2 Ra8-b8 22. Qe1-f2 Bc8-b7 23. Nb3-d4 Bb7-a8 24. a2-a4 Rb8-f8 25. Nd4xc6 Ba8xc6 26. Bf1-b5 Qg5-h5 27. h2-h3 Bc6xb5 28. a4xb5 Rf8-f6 29. Rd1-a1 Rh6-g6 30. Ra1xa7 h7-h6 31. Qf2-f1 Rg6-g3 32. Rd2-f2 Rf6-g6 33. Ra7-a3 Rg6-g5 34. c2-c4 Qh5-h4 35. Ra3-c3 Ne5-g6 36. e4-e5 Ng6xe5 37. Nd6-e4 Rg3xg2 38. Rf2xg2 Qh4xh3+ 39. Kh1-g1 Rg5-h5 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 g7-g6 3. d2-d4 Bf8-g7 4. d4xc5 Qd8-a5+ 5. Nb1-c3 Ng8-f6 6. Bf1-d3 Qa5xc5 7. O-O d7-d6 8. Bc1-e3 Qc5-a5 9. Qd1-d2 Nb8-c6 10. Be3-h6 O-O 11. Bh6xg7 Kg8xg7 12. Nc3-d5 Qa5xd2 13. Nf3xd2 Bc8-e6 14. Nd5-c7 Ra8-c8 15. Nc7xe6+ f7xe6 16. c2-c3 e6-e5 17. Rf1-e1 Nc6-d8 18. Bd3-c4 Nf6-d7 19. f2-f3 Nd7-c5 20. Re1-d1 h7-h5 21. h2-h4 g6-g5 22. h4xg5 Kg7-g6 23. g2-g3 Kg6xg5 24. Kg1-f2 Kg5-g6 25. Kf2-e3 e7-e6 26. Rd1-h1 Nc5-d7 27. Bc4-f1 Nd7-f6 28. Bf1-h3 Rc8-c7 29. Ra1-f1 d6-d5 30. f3-f4 e5xf4+ 31. Rf1xf4 d5xe4 32. Ke3-e2 e6-e5 33. Rf4-f2 Nd8-f7 34. Bh3-f5+ Kg6-h6 35. Nd2xe4 Nf7-g5 36. Ne4xf6 Rf8xf6 37. g3-g4 Ng5-f7 38. Rh1xh5+ Kh6-g7 39. Rh5-h7+ Kg7-g8 40. Bf5-e4 Rf6xf2+ 41. Ke2xf2 Rc7-c4 42. Rh7xf7 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 a7-a6 6. Bc1-e3 e7-e5 7. Nd4-b3 Bc8-e6 8. f2-f3 Bf8-e7 9. Qd1-d2 O-O 10. g2-g4 d6-d5 11. e4xd5 Nf6xd5 12. Nc3xd5 Be6xd5 13. Bf1-g2 Be7-h4+ 14. Be3-f2 Bd5xb3 15. Qd2xd8 Bh4xf2+ 16. Ke1xf2 Rf8xd8 17. Rh1-d1 Nb8-c6 18. a2xb3 1/2-1/2",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 d7-d6 8. c2-c3 O-O 9. h2-h3 Nf6-d7 10. d2-d4 Be7-f6 11. a2-a4 Bc8-b7 12. Nb1-a3 e5xd4 13. c3xd4 Rf8-e8 14. Qd1-d2 Re8-e7 15. Bb3-c2 Qd8-e8 16. b2-b3 b5xa4 17. b3xa4 a6-a5 18. Bc2-b1 Nd7-c5 19. e4-e5 d6xe5 20. Qd2-c2 e5-e4 21. Bc1-g5 Bf6xg5 22. Nf3xg5 Nc5-d3 23. Re1xe4 Re7xe4 24. Qc2xd3 Re4-e1+ 25. Kg1-h2 g7-g6 26. Bb1-a2 Re1-e7 27. Ra1-e1 Nc6-d8 28. Re1-b1 Qe8-c6 29. d4-d5 Qc6-d6+ 30. Kh2-g1 Qd6-f4 31. h3-h4 Qf4xh4 32. Qd3-d2 Bb7-a6 33. Na3-b5 Ba6xb5 34. Rb1xb5 Qh4xa4 35. Rb5-b3 Ra8-a6 36. Rb3-b8 Ra6-d6 37. Rb8-a8 Qa4-b4 38. Qd2xb4 a5xb4 39. f2-f3 Kg8-g7 40. Ng5-e4 b4-b3 41. Ba2xb3 Rd6-b6 42. Bb3-a4 Nd8-b7 43. Ba4-c6 Nb7-d6 0-1",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 d7-d6 8. c2-c3 O-O 9. h2-h3 Nc6-a5 10. Bb3-c2 c7-c5 11. d2-d4 Qd8-c7 12. Nb1-d2 Bc8-d7 13. Nd2-f1 c5xd4 14. c3xd4 Ra8-c8 15. Bc2-b1 Rf8-e8 16. d4-d5 g7-g6 17. Nf1-g3 Kg8-g7 18. Bb1-d3 Be7-d8 19. Qd1-e2 Qc7-a7 20. b2-b4 Na5-c4 21. a2-a4 Qa7-b7 22. a4-a5 Nf6-g8 23. Nf3-d2 Nc4xd2 24. Qe2xd2 h7-h6 25. Bc1-b2 Bd8-f6 26. Re1-f1 Re8-f8 27. Ra1-a3 Rc8-e8 28. Bd3-b1 Qb7-b8 29. Kg1-h2 Kg7-h7 30. f2-f4 e5xf4 31. Bb2xf6 f4xg3+ 32. Ra3xg3 Ng8xf6 33. Rf1xf6 Re8-e5 34. Qd2-f2 Kh7-g7 35. Rg3-f3 Bd7-f5 36. Rf6xf5 g6xf5 37. Qf2-g3+ Kg7-h8 38. e4xf5 Rf8-g8 39. Qg3-h4 Qb8-f8 40. f5-f6 Re5-e2 41. Rf3-f2 Re2xf2 42. Qh4xf2 Qf8-e8 43. Qf2-c2 Rg8-g6 44. Qc2-c3 Rg6-g5 45. Qc3-c2 Qe8-g8 46. h3-h4 Rg5-g4 47. g2-g3 Rg4xb4 48. Qc2-d2 1/2-1/2",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 Ng8-f6 4. e4-e5 Nf6-d7 5. c2-c3 c7-c5 6. f2-f4 Nb8-c6 7. Nd2-f3 Qd8-b6 8. g2-g3 Bf8-e7 9. Bf1-h3 c5xd4 10. c3xd4 f7-f6 11. Ke1-f1 f6xe5 12. f4xe5 O-O 13. Kf1-g2 Nd7xe5 14. d4xe5 Nc6xe5 15. Qd1-e2 Ne5-c4 16. b2-b3 Be7-f6 17. Ra1-b1 Kg8-h8 18. Qe2-f2 d5-d4 19. Qf2-e2 Nc4-d6 20. Bc1-a3 Bc8-d7 21. Bh3-g4 Ra8-c8 22. Ng1-h3 h7-h5 23. Bg4xh5 Nd6-f5 24. Rh1-c1 Nf5-e3+ 25. Kg2-h1 e6-e5 26. Ba3xf8 Rc8xf8 27. Nh3-f2 Bd7-f5 28. Nf3-d2 Bf5xb1 29. Rc1xb1 Bf6-g5 30. Bh5-f3 Qb6-c7 31. Bf3-e4 Rf8-c8 32. Nd2-f3 Bg5-f6 33. h2-h4 Qc7-d7 34. Nf3-h2 Qd7-f7 35. Nh2-g4 Rc8-c3 36. Ng4xe3 Rc3xe3 37. Qe2-g4 Bf6-e7 38. Kh1-g2 Be7-f8 39. Be4-g6 1-0",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Nb8-d7 5. Bf1-c4 Ng8-f6 6. Ne4-g5 e7-e6 7. Qd1-e2 Nd7-b6 8. Bc4-b3 h7-h6 9. Ng5-f3 a7-a5 10. a2-a3 c6-c5 11. Bc1-e3 Qd8-c7 12. Nf3-e5 a5-a4 13. Bb3-a2 c5xd4 14. Be3xd4 Bf8-c5 15. O-O-O O-O 16. Ng1-f3 Nb6-d5 17. Bd4xc5 Qc7xc5 18. Ne5-d3 Qc5-e7 19. c2-c4 Nd5-c7 20. Nf3-d4 Rf8-d8 21. Nd3-e5 Bc8-d7 22. g2-g4 Qe7-c5 23. g4-g5 h6xg5 24. h2-h4 g5-g4 25. Ne5xg4 Nc7-e8 26. Ng4-e5 Bd7-c6 27. Nd4xc6 b7xc6 28. Ba2-b1 Rd8-d5 29. f2-f4 Ra8-d8 30. Bb1-c2 Rd5xd1+ 31. Rh1xd1 Rd8-d4 32. Kc1-b1 Rd4xd1+ 33. Bc2xd1 Qc5-g1 34. Kb1-a2 c6-c5 35. Bd1xa4 Ne8-d6 36. Qe2-d1 Qg1xd1 37. Ba4xd1 Nf6-e4 38. a3-a4 f7-f6 39. Ne5-d7 Kg8-f7 40. a4-a5 Nd6-c8 41. Bd1-f3 Kf7-e7 42. Nd7-b6 Ne4-d6 43. b2-b4 Ke7-d8 44. b4xc5 Nd6-e8 45. Nb6xc8 Kd8xc8 46. Ka2-b3 Ne8-c7 47. Kb3-b4 Kc8-d7 48. Bf3-b7 Nc7-e8 49. Kb4-b5 Kd7-c7 50. Bb7-c6 1-0",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Nb8-d7 5. Ne4-g5 Ng8-f6 6. Bf1-d3 e7-e6 7. Ng1-f3 Bf8-e7 8. Ng5xf7 Ke8xf7 9. Nf3-g5+ Kf7-g8 10. Ng5xe6 Be7-b4+ 11. Ke1-f1 Qd8-e7 12. Ne6-c7 Nd7-b6 13. Nc7xa8 Nb6xa8 14. c2-c3 Bb4-d6 15. Bc1-g5 Bc8-e6 16. c3-c4 h7-h6 17. Bg5-d2 Kg8-f7 18. Qd1-c2 Rh8-f8 19. Ra1-e1 Qe7-d7 20. h2-h3 Kf7-g8 21. Kf1-e2 b7-b5 22. b2-b3 Na8-c7 23. Ke2-d1 b5xc4 24. Bd3xc4 Nf6-d5 25. g2-g4 Rf8xf2 26. Rh1-f1 Rf2-h2 27. Qc2-e4 Rh2xd2+ 28. Kd1xd2 Bd6-b4+ 29. Kd2-c1 Bb4xe1 30. Rf1xe1 Qd7-d6 31. Kc1-b2 Qd6-b4 32. Re1-c1 Qb4-d6 33. Qe4-e5 Qd6xe5 34. d4xe5 Kg8-f7 35. Rc1-f1+ Kf7-e7 36. Rf1-f3 g7-g5 37. Rf3-f2 Nd5-f4 38. Rf2-f3 Be6-d5 39. Bc4xd5 Nc7xd5 40. a2-a3 Ke7-e6 41. b3-b4 Ke6xe5 42. Kb2-b3 Ke5-d4 43. Rf3-f1 Nd5-e3 44. Rf1-c1 Kd4-d5 45. Rc1-c5+ Kd5-d6 46. Rc5-c3 Ne3-d5 47. Rc3-f3 Kd6-e5 48. Rf3-f1 a7-a6 49. Rf1-e1+ Ke5-d6 50. Re1-e8 Nf4xh3 51. Re8-h8 Nh3-f4 52. Rh8xh6+ Nf4-e6 53. Rh6-h3 Kd6-e5 54. Rh3-h7 Ke5-f4 55. Rh7-d7 Kf4xg4 56. Rd7-d6 Kg4-f5 57. Kb3-a4 Nd5-b6+ 58. Ka4-b3 Kf5-e5 59. Rd6-d1 g5-g4 60. Rd1-e1+ Ke5-f5 61. Re1-f1+ Ne6-f4 62. Rf1-g1 Nf4-e2 63. Rg1-g2 Ne2-d4+ 64. Kb3-c3 Nd4-f3 65. Rg2-a2 g4-g3 66. a3-a4 Nb6xa4+ 67. Kc3-c4 Kf5-e4 68. Ra2-e2+ Ke4-f4 69. Re2-a2 Kf4-e3 70. Ra2-a3+ Ke3-e4 71. Ra3-a2 Nf3-e5+ 72. Kc4-b3 Na4-b6 73. Ra2-e2+ Ke4-f4 74. Re2-c2 Ne5-f3 75. Rc2-a2 Kf4-g4 76. Kb3-c3 Kg4-h3 77. Ra2xa6 g3-g2 78. Ra6-a1 Nb6-d5+ 79. Kc3-d3 Nd5xb4+ 80. Kd3-c4 Nb4-c2 0-1",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-c3 Bf8-b4 4. e4-e5 b7-b6 5. Bc1-d2 Qd8-d7 6. Bf1-b5 c7-c6 7. Bb5-a4 a7-a5 8. a2-a3 Bb4-f8 9. Ng1-f3 Bc8-a6 10. Nc3-e2 Ng8-e7 11. O-O Ba6-b5 12. Ba4xb5 c6xb5 13. Ne2-c3 Nb8-a6 14. a3-a4 b5xa4 15. Nc3xa4 Ne7-c8 16. b2-b3 Bf8-e7 17. Qd1-e2 O-O 18. Rf1-c1 Qd7-b7 19. c2-c4 Na6-b4 20. c4-c5 b6xc5 21. Na4xc5 Be7xc5 22. Rc1xc5 Nb4-c6 23. Qe2-d3 h7-h6 24. Bd2xa5 Nc8-e7 25. Ra1-a4 Rf8-b8 26. Ba5-d2 Ra8-a7 27. Ra4xa7 Nc6xa7 28. Rc5-c3 Na7-b5 29. Rc3-c1 Ne7-c6 30. h2-h3 Qb7-b6 31. Bd2-e3 Nb5-a7 32. Rc1-c3 Nc6-a5 33. Nf3-h2 Na5xb3 34. Nh2-g4 Nb3-a5 35. Kg1-h2 Qb6-b1 36. Qd3-a6 Qb1-b6 37. Qa6-e2 Kg8-f8 38. Ng4xh6 Na7-c6 39. Nh6-g4 Qb6-b2 40. Qe2-d3 Qb2-b1 41. Rc3-c2 Kf8-g8 42. Qd3-e2 Na5-c4 43. h3-h4 Kg8-f8 44. h4-h5 Kf8-e7 45. h5-h6 g7xh6 46. Ng4-f6 Ke7-d8 47. Rc2xc4 d5xc4 48. Qe2xc4 Qb1-b5 49. Qc4-c2 Kd8-c7 50. Qc2-h7 Nc6-d8 51. Qh7xh6 Qb5-c6 52. Qh6-f8 Kc7-b7 53. Qf8-e7+ Qc6-c7 54. Nf6-d7 Kb7-c8 0-1",
    "1. c2-c4 e7-e5 2. g2-g3 Ng8-f6 3. Bf1-g2 Nb8-c6 4. Ng1-f3 g7-g6 5. d2-d4 e5xd4 6. Nf3xd4 Bf8-g7 7. Nb1-c3 O-O 8. O-O Rf8-e8 9. e2-e3 Ra8-b8 10. b2-b3 Nc6xd4 11. e3xd4 d7-d6 12. h2-h3 a7-a6 13. Bc1-e3 b7-b5 14. c4xb5 a6xb5 15. Ra1-c1 b5-b4 16. Nc3-e2 Bc8-b7 17. d4-d5 Bb7-a6 18. Rf1-e1 Ba6xe2 19. Qd1xe2 Nf6-d7 20. Qe2-c4 Bg7-c3 21. Re1-d1 Nd7-b6 22. Qc4-f4 Bc3-e5 23. Qf4-f3 Rb8-a8 24. Rc1-c2 Nb6-d7 25. Be3-d2 Qd8-b8 26. Qf3-g4 f7-f5 27. Qg4xb4 Qb8xb4 28. Bd2xb4 Re8-b8 29. Bb4-e1 Rb8-b7 30. Bg2-f1 Nd7-c5 31. Bf1-c4 Kg8-f8 32. b3-b4 Nc5-e4 33. Bc4-b3 Be5-f6 34. f2-f3 Ne4-g5 35. Kg1-g2 Kf8-e7 36. a2-a4 Ke7-d8 37. Rc2-e2 Ng5-f7 38. f3-f4 Kd8-d7 39. Bb3-c2 Nf7-d8 40. a4-a5 Ra8-b8 41. Bc2-a4+ Kd7-c8 42. Ba4-c6 Rb7xb4 43. Be1xb4 Rb8xb4 44. Re2-e8 Rb4-b2+ 45. Kg2-f3 Kc8-b8 46. Re8-f8 Bf6-e7 47. Rf8-h8 Rb2-b3+ 48. Kf3-e2 h7-h5 49. Rd1-d3 Rb3-b2+ 50. Ke2-d1 Be7-f6 51. Rh8-f8 Bf6-e7 52. Rf8-e8 Be7-f6 53. Rd3-e3 h5-h4 54. Re3-e6 Kb8-a7 55. Re6xf6 1-0",
    "1. c2-c4 e7-e5 2. Nb1-c3 Ng8-f6 3. Ng1-f3 Nb8-c6 4. e2-e3 Bf8-b4 5. Nc3-d5 e5-e4 6. Nf3-g1 O-O 7. a2-a3 Bb4-d6 8. Qd1-c2 Rf8-e8 9. Ng1-e2 b7-b5 10. b2-b3 b5xc4 11. b3xc4 Nf6xd5 12. c4xd5 Nc6-e5 13. Ne2-g3 Bc8-b7 14. Ng3xe4 Bb7xd5 15. Ne4xd6 c7xd6 16. Qc2-f5 g7-g6 17. Qf5-h3 Qd8-f6 18. Ra1-b1 Bd5-e6 19. Qh3-g3 Qf6-f5 20. Rb1-a1 Ra8-b8 21. Bf1-a6 Rb8-b6 22. f2-f4 Ne5-c6 23. Ba6-e2 Qf5-f6 24. d2-d4 Nc6xd4 25. e3xd4 Be6-c4 26. Qg3-c3 Re8xe2+ 27. Ke1-d1 Qf6-e6 28. Rh1-e1 Bc4-b3+ 29. Qc3xb3 Re2xe1+ 0-1",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4xd5 c6xd5 4. c2-c4 Ng8-f6 5. Nb1-c3 e7-e6 6. Ng1-f3 Bf8-b4 7. Bc1-d2 d5xc4 8. Bf1xc4 O-O 9. O-O Nb8-d7 10. Qd1-b3 Bb4-e7 11. Rf1-e1 Nd7-b6 12. Bc4-d3 Bc8-d7 13. Nf3-e5 Bd7-e8 14. Bd2-e3 Nf6-d5 15. Nc3xd5 Qd8xd5 16. Qb3xd5 Nb6xd5 17. Bd3-e4 f7-f6 18. Be4xd5 e6xd5 19. Ne5-d3 Be8-g6 20. Be3-f4 Rf8-f7 21. Re1-e3 b7-b6 22. Ra1-e1 Be7-f8 23. Bf4-g3 Rf7-d7 24. Nd3-f4 Bg6-f7 25. a2-a3 Ra8-c8 26. f2-f3 Rc8-c4 27. Re3-d3 Rc4-c2 28. Re1-e2 Rc2-c6 29. Rd3-e3 Rc6-c4 30. Re2-d2 g7-g5 31. Nf4-e2 b6-b5 32. Re3-c3 a7-a5 33. Bg3-f2 b5-b4 34. Rc3xc4 d5xc4 35. a3xb4 a5xb4 36. Ne2-g3 Bf7-d5 37. Ng3-e4 Kg8-f7 38. Rd2-c2 Rd7-a7 39. Bf2-e1 Ra7-a2 40. Rc2-d2 g5-g4 41. Kg1-f2 g4xf3 42. g2xf3 f6-f5 43. Ne4-c5 Bf8xc5 44. d4xc5 c4-c3 45. b2xc3 b4-b3 46. c3-c4 Ra2xd2+ 0-1",
    "1. e2-e4 c7-c5 2. Ng1-f3 Nb8-c6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 d7-d6 6. Bc1-e3 e7-e6 7. Qd1-d2 Bf8-e7 8. f2-f3 O-O 9. g2-g4 Nc6xd4 10. Be3xd4 Nf6-d7 11. O-O-O a7-a6 12. h2-h4 b7-b5 13. g4-g5 Qd8-a5 14. Kc1-b1 b5-b4 15. Nc3-e2 Nd7-e5 16. Rh1-h3 Bc8-d7 17. Ne2-c1 Ra8-c8 18. Rh3-g3 Ne5-c4 19. Bf1xc4 Rc8xc4 20. Bd4-f6 Be7-d8 21. Bf6xd8 Rf8xd8 22. h4-h5 Bd7-e8 23. Rg3-g2 Qa5-c7 24. h5-h6 g7-g6 25. Nc1-d3 a6-a5 26. Nd3-e5 Rc4-c5 27. Ne5-g4 f7-f5 28. Ng4-f6+ Kg8-f8 29. Rg2-f2 Be8-a4 30. b2-b3 Ba4-c6 31. Rd1-e1 Qc7-f7 32. e4xf5 Rc5xf5 33. f3-f4 Bc6-d5 34. Qd2-d4 Qf7-b7 35. Rf2-e2 Qb7-f7 36. Nf6xd5 Rf5xd5 37. Qd4-h8+ Kf8-e7 38. Re2xe6+ Ke7-d7 39. Re6-e7+ Qf7xe7 40. Re1xe7+ Kd7xe7 41. Qh8xh7+ Ke7-e6 42. Qh7xg6+ Ke6-d7 43. h6-h7 Rd5-d1+ 44. Kb1-b2 Rd1-h1 45. Qg6-f5+ 1-0",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4-e5 Bc8-f5 4. Nb1-c3 Qd8-b6 5. h2-h4 h7-h5 6. Ng1-e2 e7-e6 7. Ne2-f4 c6-c5 8. Bf1-b5+ Nb8-c6 9. Bc1-e3 O-O-O 10. Bb5xc6 Qb6xc6 11. Nf4xh5 Qc6-b6 12. Nh5-g3 Qb6xb2 13. Nc3-e2 Bf5xc2 14. Qd1-c1 Qb2xc1+ 15. Ra1xc1 Bc2-d3 16. d4xc5 Ng8-e7 17. Ne2-d4 Ne7-g6 18. Be3-g5 Rd8-e8 19. f2-f4 Bf8-e7 20. Bg5xe7 Re8xe7 21. Ke1-d2 Bd3-c4 22. f4-f5 Ng6xe5 23. f5xe6 f7xe6 24. Rc1-e1 Ne5-g6 25. h4-h5 Ng6-f4 26. Ng3-f5 Re7-c7 27. Nd4xe6 Nf4xe6 28. Re1xe6 Rc7xc5 29. Re6-e7 Kc8-b8 30. Re7xg7 d5-d4 31. Nf5-d6 Bc4-d5 32. h5-h6 Rc5-c6 33. Nd6-f5 Bd5-e4 34. Nf5xd4 Rc6xh6 35. Rh1xh6 Rh8xh6 36. Nd4-b5 1/2-1/2",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Nb8-c6 5. Nb1-c3 Qd8-c7 6. Bf1-e2 a7-a6 7. O-O Ng8-f6 8. Kg1-h1 Nc6xd4 9. Qd1xd4 Bf8-c5 10. Qd4-d3 h7-h5 11. f2-f4 Nf6-g4 12. Nc3-d1 b7-b5 13. Nd1-e3 Ng4xe3 14. Bc1xe3 Bc5xe3 15. Qd3xe3 Bc8-b7 16. c2-c3 h5-h4 17. h2-h3 Ra8-c8 18. Qe3-d4 O-O 19. e4-e5 Qc7-c5 20. Ra1-d1 Rf8-d8 21. Qd4xc5 Rc8xc5 22. Rd1-d6 a6-a5 23. Be2-f3 Bb7xf3 24. Rf1xf3 b5-b4 25. c3xb4 Rc5-c1+ 26. Kh1-h2 a5xb4 27. Rd6-d4 Rd8-a8 28. Rd4xb4 Ra8xa2 29. f4-f5 Ra2-a1 30. Rb4xh4 Rc1-c5 31. Rf3-d3 e6xf5 32. Rd3xd7 Ra1-a8 33. Rh4-b4 Rc5xe5 34. Rb4-b7 Ra8-f8 35. b2-b4 f5-f4 36. b4-b5 Re5-e2 37. Kh2-g1 f4-f3 38. g2xf3 Rf8-a8 39. Rb7-a7 Ra8-c8 1/2-1/2",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O b7-b5 6. Ba4-b3 Bc8-b7 7. d2-d3 Bf8-c5 8. a2-a4 d7-d6 9. Nb1-c3 b5-b4 10. Nc3-d5 Nc6-a5 11. Nd5xf6+ Qd8xf6 12. Bb3-a2 Bb7-c8 13. Bc1-g5 Qf6-g6 14. Bg5-e3 Bc8-g4 15. Be3xc5 Bg4xf3 16. Qd1xf3 d6xc5 17. Qf3-e3 Qg6-d6 18. f2-f4 O-O 19. f4xe5 Qd6xe5 20. Ra1-b1 Ra8-e8 21. Rf1-f3 c5-c4 22. d3xc4 c7-c5 23. Qe3-f2 g7-g6 24. Rb1-f1 Re8-e7 25. Rf3-f6 Na5-b7 26. Rf6xa6 Nb7-d6 27. Qf2-f6 Qe5xf6 28. Rf1xf6 Nd6xe4 29. Rf6-f1 Ne4-d2 30. Rf1-a1 f7-f5 31. Ra6-d6 Re7-e2 32. Rd6-d3 f5-f4 33. Ra1-d1 Nd2-e4 34. Ba2-b3 g6-g5 35. Kg1-f1 Re2-f2+ 36. Kf1-g1 Rf2-e2 37. h2-h3 Rf8-e8 38. Rd3-d5 h7-h5 39. a4-a5 g5-g4 40. h3xg4 h5xg4 41. Rd1-f1 Re8-f8 42. Rd5-d3 Ne4-g5 43. Rd3-d5 Ng5-e4 44. Rd5-d3 Kg8-g7 45. c2-c3 b4xc3 46. b2xc3 Re2-d2 47. Rf1-d1 Rd2-b2 48. Rd1-e1 Ne4-f2 49. Rd3-d7+ Kg7-g6 50. Re1-f1 g4-g3 51. Rd7-d6+ Kg6-g7 52. Rd6-d7+ Kg7-f6 53. Rd7-d6+ Kf6-e7 54. Rd6-h6 Rb2xb3 55. a5-a6 Rf8-f6 56. Rh6-h7+ Rf6-f7 57. Rh7-h6 Rb3-a3 58. Rf1-b1 Ra3-a2 59. Kg1-f1 Nf2-g4 60. Rh6-g6 Ng4-e3+ 61. Kf1-e1 Ne3xg2+ 62. Ke1-f1 Ng2-e3+ 63. Kf1-e1 Ne3-c2+ 64. Ke1-f1 g3-g2+ 65. Kf1-g1 Nc2-b4 66. c3xb4 0-1",
    "1. e2-e4 d7-d6 2. d2-d4 Ng8-f6 3. Nb1-c3 c7-c6 4. Ng1-f3 Bc8-g4 5. a2-a4 e7-e6 6. Bf1-e2 d6-d5 7. e4-e5 Nf6-d7 8. O-O c6-c5 9. Nc3-b5 Bg4xf3 10. Be2xf3 c5xd4 11. Bc1-f4 Nb8-c6 12. Rf1-e1 Bf8-c5 13. Bf3xd5 e6xd5 14. e5-e6 f7xe6 15. Nb5-c7+ Ke8-f7 16. Nc7xe6 Qd8-f6 17. Qd1-h5+ Kf7-g8 18. Qh5xd5 Qf6-f7 19. Bf4-d6 Bc5-b6 20. a4-a5 Nd7-f6 21. Qd5-b5 Bb6xa5 22. Ne6-g5 Qf7-d5 23. Qb5xb7 Ra8-d8 24. Ra1xa5 Nc6xa5 25. Qb7-c7 Na5-c6 26. Re1-e5 Nf6-e8 27. Re5xe8+ Rd8xe8 28. h2-h3 h7-h6 0-1",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 a7-a6 6. Bc1-e3 e7-e6 7. Qd1-d2 b7-b5 8. f2-f3 Bc8-b7 9. g2-g4 h7-h6 10. O-O-O Nb8-d7 11. Bf1-d3 b5-b4 12. Nc3-e2 d6-d5 13. e4xd5 Nf6xd5 14. Ne2-f4 Qd8-a5 15. Kc1-b1 Nd5xe3 16. Qd2xe3 Qa5-e5 17. Qe3-c1 Qe5xd4 18. Nf4xe6 Qd4-b6 19. Bd3-g6 f7xg6 20. Rh1-e1 Nd7-f6 21. Qc1-f4 Bf8-e7 22. Ne6xg7+ Ke8-f8 23. Rd1-d7 Rh8-h7 24. Rd7xe7 Rh7xg7 25. Qf4xh6 Nf6-e8 26. Qh6-h8+ Rg7-g8 27. Qh8-e5 Ne8-g7 28. Re7-f7+ 1-0",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Nb8-d7 5. Bf1-c4 Ng8-f6 6. Ne4xf6+ Nd7xf6 7. c2-c3 Qd8-c7 8. Ng1-f3 Bc8-g4 9. Qd1-b3 e7-e6 10. Nf3-e5 Bg4-f5 11. O-O Bf8-d6 12. Rf1-e1 O-O 13. h2-h3 Ra8-c8 14. Bc1-f4 h7-h6 15. Ra1-d1 Nf6-d5 16. Bf4-h2 Rf8-d8 17. Bc4-f1 Qc7-b6 18. Ne5-c4 Bd6xh2+ 19. Kg1xh2 Qb6-c7+ 20. Kh2-g1 Nd5-b6 21. Nc4-e3 Bf5-g6 22. Qb3-a3 Nb6-d5 23. Ne3-c4 b7-b6 24. Nc4-e5 Bg6-f5 25. b2-b4 Qc7-e7 26. Qa3-b3 Rc8-c7 27. a2-a3 Nd5-f6 28. c3-c4 Nf6-d7 29. Ne5-f3 Qe7-f8 30. c4-c5 Nd7-f6 31. Nf3-e5 Bf5-e4 32. Qb3-g3 Qf8-e7 33. Ne5-c4 b6-b5 34. Nc4-d6 Be4-g6 35. Bf1-d3 Bg6xd3 36. Rd1xd3 Nf6-d5 37. Rd3-f3 Rd8-f8 38. Re1-e5 Qe7-d8 39. Re5-h5 Kg8-h7 40. Rh5-e5 Rc7-d7 41. Re5-e4 Nd5-f6 42. Re4-e2 Nf6-e8 43. Re2xe6 Ne8xd6 44. Re6xd6 Rd7xd6 45. c5xd6 Qd8-d7 46. d4-d5 c6xd5 47. Qg3-e5 Rf8-d8 48. Qe5xd5 Qd7xd6 49. Qd5xd6 Rd8xd6 50. Rf3xf7 a7-a6 51. Rf7-a7 h6-h5 52. h3-h4 Kh7-g6 53. f2-f3 Kg6-f6 54. Kg1-f2 Rd6-c6 55. Kf2-e2 g7-g6 56. Ke2-d3 Kf6-e5 57. g2-g3 Rc6-d6+ 58. Kd3-e3 Rd6-c6 59. Ke3-d3 Rc6-d6+ 60. Kd3-e3 Rd6-c6 61. Ra7-e7+ Ke5-f6 62. Re7-d7 Kf6-e6 63. Rd7-d3 Ke6-e5 64. Ke3-d2 Ke5-f6 65. Rd3-c3 Rc6-e6 66. Kd2-d3 g6-g5 67. h4xg5+ Kf6xg5 68. Kd3-d4 Re6-d6+ 69. Kd4-c5 Rd6-d1 70. Kc5-b6 Rd1-g1 71. Kb6xa6 Rg1xg3 72. Ka6xb5 h5-h4 73. a3-a4 h4-h3 74. Rc3-c1 Kg5-f4 75. a4-a5 Rg3-g8 76. a5-a6 h3-h2 77. Rc1-h1 1-0",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 c7-c5 4. e4xd5 e6xd5 5. Ng1-f3 Nb8-c6 6. Bf1-b5 Bf8-d6 7. O-O Ng8-e7 8. d4xc5 Bd6xc5 9. Nd2-b3 Bc5-d6 10. Nf3-d4 O-O 11. c2-c3 Nc6-e5 12. Bb5-e2 f7-f5 13. Bc1-f4 Ne7-g6 14. Bf4xe5 Bd6xe5 15. Rf1-e1 a7-a6 16. Qd1-c2 Be5-c7 17. Ra1-d1 Ng6-f4 18. Be2-f1 Qd8-d6 19. g2-g3 Nf4-h5 20. f2-f4 g7-g6 21. c3-c4 Nh5-f6 22. Kg1-h1 Nf6-e4 23. Bf1-g2 Bc8-d7 24. c4xd5 Ra8-c8 25. Re1-e2 Rf8-e8 26. Qc2-b1 Bc7-b6 27. Bg2-f3 Qd6xd5 28. Kh1-g2 Qd5-c4 29. Re2-c2 Qc4-b4 30. Rc2xc8 Re8xc8 31. Qb1-d3 Qb4-c4 32. Qd3-b1 Qc4-b4 33. h2-h4 a6-a5 34. Nd4-e2 Bd7-b5 35. Nb3-d4 Ne4-d2 36. Rd1xd2 Qb4xd2 37. Nd4xb5 Rc8-c2 38. a2-a4 Rc2xb2 39. Qb1-a1 Bb6-c5 40. Kg2-h3 b7-b6 41. h4-h5 Bc5-f8 42. Nb5-d4 Bf8-g7 43. Qa1-a3 Qd2-b4 44. Qa3-e3 Qb4-d6 45. Qe3-e8+ Qd6-f8 46. Qe8-d7 Rb2-d2 47. Bf3-d5+ Kg8-h8 48. Nd4-e6 Qf8-g8 49. Ne6-g5 Qg8-f8 50. h5-h6 1-0",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. g2-g3 Bf8-b4+ 4. Bc1-d2 Qd8-e7 5. Ng1-f3 Nb8-c6 6. Nb1-c3 O-O 7. Bf1-g2 Bb4xc3 8. Bd2xc3 Nf6-e4 9. Ra1-c1 d7-d6 10. d4-d5 Ne4xc3 11. Rc1xc3 Nc6-b8 12. d5xe6 f7xe6 13. O-O Nb8-d7 14. Qd1-c2 a7-a5 15. Nf3-d2 Ra8-a7 16. h2-h4 h7-h6 17. Rc3-e3 Kg8-h8 18. Bg2-h3 Nd7-c5 19. Nd2-e4 b7-b6 20. Ne4-c3 c7-c6 21. Rf1-d1 Qe7-d8 22. Re3-f3 Rf8xf3 23. e2xf3 Ra7-f7 24. f3-f4 d6-d5 25. Nc3-e2 Qd8-c7 26. Rd1-e1 Bc8-a6 27. Ne2-d4 d5xc4 28. Nd4xe6 Qc7-e7 29. Re1-e3 Nc5-d3 30. Ne6-g5 h6xg5 31. Re3xe7 Rf7xe7 32. f4xg5 c6-c5 33. Qc2-d2 a5-a4 34. g5-g6 Re7-e5 35. Bh3-f1 a4-a3 36. b2xa3 Ba6-b7 37. g3-g4 Bb7-c6 38. Bf1xd3 c4xd3 39. Qd2xd3 Re5-d5 40. Qd3-f3 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 Nb8-c6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 d7-d6 6. Bf1-e2 Nf6-g4 7. Bc1-g5 Qd8-b6 8. Be2-b5 Bc8-d7 9. O-O h7-h6 10. Bg5-e3 Ng4xe3 11. f2xe3 e7-e6 12. Bb5xc6 b7xc6 13. Qd1-f3 f7-f6 14. Qf3-h5+ Ke8-e7 15. Ra1-d1 Bd7-e8 16. Qh5-h3 Be8-f7 17. b2-b4 Ra8-d8 18. Qh3-g3 Ke7-e8 19. Qg3-h3 e6-e5 20. Nd4-e6 Bf7xe6 21. Qh3xe6+ Bf8-e7 22. Rd1-d3 Qb6-b7 23. a2-a4 Qb7-d7 24. Qe6-b3 Rh8-f8 25. b4-b5 c6xb5 26. a4xb5 f6-f5 27. e4xf5 Rf8xf5 28. Rd3-d1 Rf5xf1+ 29. Rd1xf1 d6-d5 30. e3-e4 Be7-c5+ 31. Kg1-h1 Qd7-d6 32. Nc3xd5 Rd8-d7 33. Nd5-c3 Ke8-d8 34. Qb3-g8+ Kd8-c7 35. Qg8-a8 Kc7-b6 36. Rf1-a1 Bc5-a3 37. Qa8-c8 1-0",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. g2-g3 Bf8-b4+ 4. Bc1-d2 Qd8-e7 5. Bf1-g2 O-O 6. Ng1-f3 Bb4xd2+ 7. Qd1xd2 d7-d6 8. O-O e6-e5 9. Nb1-c3 Rf8-e8 10. e2-e4 Bc8-g4 11. Nf3-e1 e5xd4 12. Qd2xd4 Nb8-c6 13. Qd4-d2 a7-a5 14. Ra1-c1 Qe7-d8 15. b2-b3 a5-a4 16. f2-f3 a4xb3 17. f3xg4 b3xa2 18. Nc3xa2 Nf6xg4 19. Na2-c3 Ng4-e5 20. Nc3-d5 Nc6-a5 21. Qd2-e2 c7-c6 22. Nd5-e3 Na5-b3 23. Rc1-d1 Nb3-c5 24. Kg1-h1 Ra8-a4 25. Ne3-f5 Re8-e6 26. Ne1-f3 Ne5xc4 27. Nf3-d4 Re6-f6 28. e4-e5 Nc4xe5 29. Nd4-b5 Qd8-f8 30. Nb5xd6 Rf6-e6 31. Qe2-c2 b7-b6 32. Qc2-b1 b6-b5 33. Qb1-c2 Ne5-c4 34. Nd6xf7 Ra4-a8 35. Nf7-g5 Re6-g6 36. Ng5xh7 Kg8xh7 37. Nf5-h4 Qf8-e8 38. Rd1-e1 Nc4-a3 39. Qc2xc5 Rg6-e6 40. Qc5-f5+ 1-0",
    "1. e2-e4 c7-c5 2. Nb1-c3 d7-d6 3. Ng1-f3 Ng8-f6 4. d2-d4 c5xd4 5. Nf3xd4 a7-a6 6. Bc1-e3 e7-e5 7. Nd4-f3 Bf8-e7 8. Bf1-c4 O-O 9. O-O Nb8-d7 10. a2-a4 Nf6-g4 11. Be3-c1 Ng4-f6 12. Qd1-e2 Qd8-c7 13. Rf1-d1 Nd7-c5 14. Bc1-g5 Bc8-e6 15. Bc4xe6 f7xe6 16. Qe2-c4 Qc7-c6 17. b2-b4 b7-b5 18. Qc4-a2 Nc5xe4 19. Qa2xe6+ Rf8-f7 20. Nf3xe5 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. c2-c3 d7-d5 4. e4xd5 e6xd5 5. d2-d4 Nb8-c6 6. Bf1-b5 Bf8-d6 7. d4xc5 Bd6xc5 8. O-O Ng8-e7 9. Nb1-d2 O-O 10. Nd2-b3 Bc5-d6 11. Bc1-g5 Qd8-c7 12. Bg5-h4 Ne7-f5 13. Bh4-g3 Nf5xg3 14. h2xg3 Rf8-d8 15. Rf1-e1 a7-a6 16. Bb5-d3 g7-g6 17. Qd1-d2 Bd6-f8 18. Qd2-f4 Qc7xf4 19. g3xf4 f7-f6 20. Nf3-d4 Kg8-f7 21. Bd3-e2 Bf8-d6 22. g2-g3 Bc8-d7 23. Be2-f3 Nc6-e7 24. Kg1-g2 a6-a5 25. a2-a4 b7-b6 26. Nd4-b5 Bd7xb5 27. a4xb5 Ra8-c8 28. Re1-e2 Rd8-d7 29. Re2-d2 Bd6-c5 30. Ra1-d1 a5-a4 31. Nb3-d4 a4-a3 32. b2xa3 Bc5xa3 33. Nd4-c6 Kf7-g7 34. Bf3xd5 Ba3-c5 35. Bd5-e6 Rd7xd2 36. Rd1xd2 Rc8-c7 37. Nc6-d8 Bc5-a3 38. c3-c4 f6-f5 39. Rd2-a2 Ba3-b4 40. Ra2-a6 Bb4-a5 41. Ra6xa5 b6xa5 42. b5-b6 Rc7-c6 43. Nd8xc6 Ne7xc6 44. Be6-d5 Nc6-b4 45. b6-b7 1-0",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Ng1-f3 b7-b6 4. a2-a3 Bc8-a6 5. Qd1-c2 Ba6-b7 6. Nb1-c3 c7-c5 7. d4xc5 b6xc5 8. Bc1-g5 Bf8-e7 9. e2-e3 d7-d6 10. Bf1-e2 Nb8-d7 11. Ra1-d1 a7-a6 12. Bg5-f4 Qd8-b6 13. Rd1-d2 h7-h6 14. h2-h3 Ra8-d8 15. O-O O-O 16. Rf1-d1 Nd7-b8 17. Nf3-e1 Rd8-d7 18. Be2-f3 Rf8-d8 19. Nc3-a4 Qb6-c7 20. Bf4-g3 Nf6-e8 21. b2-b4 c5xb4 22. a3xb4 Bb7xf3 23. Ne1xf3 Qc7-b7 24. Rd1-b1 Ne8-f6 25. Rd2-d3 Rd8-c8 26. Nf3-d2 Rd7-d8 27. Qc2-a2 Nf6-e4 28. Nd2xe4 Qb7xe4 29. Rd3-d4 Qe4-b7 30. Rd4-d1 Qb7-c6 31. c4-c5 d6xc5 32. Rd1xd8+ Rc8xd8 33. Na4xc5 Be7xc5 34. b4xc5 Nb8-d7 35. Bg3-d6 Rd8-a8 36. Qa2-a5 Kg8-h7 37. Rb1-a1 Ra8-c8 1/2-1/2",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Nb8-c6 5. Nb1-c3 a7-a6 6. Nd4xc6 b7xc6 7. Bf1-d3 d7-d6 8. O-O Ng8-f6 9. f2-f4 Qd8-c7 10. Qd1-e1 Bf8-e7 11. Kg1-h1 d6-d5 12. Qe1-g3 g7-g6 13. b2-b3 Ra8-b8 14. Qg3-f2 c6-c5 15. e4xd5 e6xd5 16. f4-f5 Rb8-b6 17. Bc1-g5 Bc8-b7 18. Ra1-e1 O-O 19. Qf2-h4 Be7-d8 20. Nc3-a4 Rb6-b4 21. c2-c4 d5xc4 22. Bd3xc4 Bb7-d5 23. Na4-c3 Bd5xc4 24. b3xc4 Rb4-b6 25. Re1-e3 h7-h5 26. h2-h3 Qc7-c6 27. Re3-f3 Rb6-b2 28. f5xg6 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 d7-d6 6. Bf1-e2 Bf8-e7 7. O-O O-O 8. f2-f4 Nb8-c6 9. Bc1-e3 Qd8-c7 10. Nd4-b5 Qc7-b8 11. a2-a4 Bc8-d7 12. Qd1-d2 a7-a6 13. Nb5-a3 Nc6-b4 14. Na3-c4 Bd7-c6 15. Nc4-b6 Ra8-a7 16. Be2-f3 Rf8-d8 17. Qd2-f2 Ra7-a8 18. Nb6xa8 Qb8xa8 19. Be3-b6 Rd8-c8 20. Kg1-h1 Bc6-e8 21. Ra1-d1 Nf6-d7 22. Bb6-d4 Nb4-c6 23. Nc3-e2 Nc6xd4 24. Ne2xd4 Qa8-a7 25. Rd1-d2 Be7-f6 26. c2-c3 Qa7-b8 27. Bf3-d1 Nd7-c5 28. Bd1-c2 b7-b5 29. a4xb5 a6xb5 30. Qf2-e3 h7-h6 31. b2-b4 Nc5-a4 32. Bc2xa4 b5xa4 33. b4-b5 e6-e5 34. f4xe5 Bf6-g5 35. Qe3-f2 Bg5xd2 1/2-1/2",
    "1. e2-e4 d7-d5 2. e4xd5 Ng8-f6 3. c2-c4 e7-e6 4. d5xe6 Bc8xe6 5. Bf1-e2 Nb8-c6 6. Ng1-f3 Bf8-c5 7. O-O Nc6-d4 8. Nf3xd4 Qd8xd4 9. d2-d3 O-O-O 10. Nb1-d2 Bc5-d6 11. Nd2-f3 Qd4-b6 12. d3-d4 c7-c5 13. d4-d5 Be6-g4 14. Nf3-g5 Bg4xe2 15. Qd1xe2 Rh8-e8 16. Qe2-d3 Rd8-d7 17. b2-b3 Nf6-g4 18. Ng5-f3 Qb6-c7 19. h2-h3 Bd6-h2+ 20. Kg1-h1 Ng4-e5 21. Qd3-e2 Ne5xf3 22. Qe2xe8+ Rd7-d8 23. Qe8-e4 Bh2-e5 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Ba4xc6 d7xc6 7. Qd1-e1 Nf6-d7 8. d2-d4 e5xd4 9. Nf3xd4 Nd7-c5 10. Qe1-e3 O-O 11. Nb1-c3 Rf8-e8 12. Rf1-d1 Be7-d6 13. b2-b4 Nc5-d7 14. Nd4-f5 Nd7-e5 15. Nf5xd6 c7xd6 16. Qe3-g3 Re8-e6 17. Kg1-h1 Qd8-b6 18. Ra1-b1 Re6-g6 19. Qg3-h4 Bc8-g4 20. Bc1-e3 Qb6-c7 21. Rd1-f1 Ne5-c4 22. Be3-d4 Nc4-d2 23. f2-f4 f7-f5 24. h2-h3 Nd2xb1 25. Nc3xb1 f5xe4 26. f4-f5 Ra8-f8 27. f5xg6 Rf8xf1+ 28. Kh1-h2 d6-d5+ 29. g2-g3 h7xg6 30. Qh4xg4 Rf1xb1 31. Qg4xg6 Rb1-f1 32. h3-h4 Qc7-f7 0-1",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4xd5 c6xd5 4. c2-c4 Ng8-f6 5. Nb1-c3 e7-e6 6. Ng1-f3 Bf8-e7 7. Bf1-d3 O-O 8. O-O d5xc4 9. Bd3xc4 Nb8-d7 10. Bc1-g5 Nd7-b6 11. Bc4-b3 Bc8-d7 12. Qd1-d3 Bd7-c6 13. Ra1-d1 Ra8-c8 14. Nf3-e5 Bc6-d5 15. Nc3xd5 Nf6xd5 16. Bg5xe7 Qd8xe7 17. f2-f4 g7-g6 18. Qd3-h3 a7-a5 19. Rd1-d3 a5-a4 20. Bb3xd5 Nb6xd5 21. Qh3-h6 Nd5-f6 22. Rd3-h3 Qe7-d6 23. Ne5-g4 Qd6xd4+ 24. Kg1-h1 Rf8-d8 25. Ng4xf6+ Qd4xf6 26. Qh6xh7+ Kg8-f8 27. Qh7-h8+ Kf8-e7 28. Qh8xf6+ Ke7xf6 29. b2-b3 Rd8-d2 30. b3xa4 Rd2xa2 31. Rh3-b3 Rc8-c2 32. Rf1-g1 Ra2xa4 33. Rb3xb7 Ra4xf4 34. h2-h3 Rf4-f2 35. Kh1-h2 Kf6-g5 36. Rb7-e7 Rc2-e2 37. Kh2-g3 f7-f5 38. h3-h4+ Kg5-f6 39. Re7-c7 e6-e5 40. Rc7-c6+ Kf6-g7 41. h4-h5 g6xh5 42. Kg3-h3 f5-f4 43. Rg1-a1 Rf2xg2 44. Ra1-a7+ Kg7-h8 45. Kh3-h4 Re2-e1 46. Rc6-c8+ Rg2-g8 47. Rc8xg8+ Kh8xg8 0-1",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 a7-a6 6. Bc1-e3 e7-e5 7. Nd4-b3 Bc8-e6 8. Qd1-d2 Nb8-d7 9. f2-f3 Bf8-e7 10. g2-g4 h7-h6 11. h2-h4 b7-b5 12. O-O-O Nd7-b6 13. Qd2-f2 Nf6-d7 14. Kc1-b1 Qd8-c7 15. Nc3-d5 Be6xd5 16. e4xd5 Nb6-c4 17. f3-f4 a6-a5 18. f4xe5 d6xe5 19. Nb3-d2 Nc4xb2 20. Bf1-d3 Nb2xd3 21. c2xd3 Nd7-f6 22. Nd2-e4 Nf6xd5 23. Rh1-g1 b5-b4 24. Rd1-f1 Be7xh4 25. Qf2-f3 Nd5xe3 26. Qf3xe3 b4-b3 27. a2xb3 Ra8-b8 28. d3-d4 O-O 29. g4-g5 h6xg5 0-1",
    "1. Ng1-f3 Ng8-f6 2. g2-g3 d7-d5 3. Bf1-g2 c7-c6 4. O-O Bc8-g4 5. c2-c4 Nb8-d7 6. c4xd5 c6xd5 7. Nb1-c3 e7-e6 8. d2-d3 Bf8-d6 9. h2-h3 Bg4-h5 10. e2-e4 O-O 11. Qd1-e2 d5xe4 12. d3xe4 Qd8-b8 13. Bc1-g5 h7-h6 14. Bg5xf6 Bh5xf3 15. Bg2xf3 Nd7xf6 16. Rf1-d1 Bd6-e5 17. Ra1-c1 Rf8-d8 18. Bf3-g2 Rd8xd1+ 19. Qe2xd1 g7-g5 20. Qd1-d2 Qb8-d8 21. Rc1-d1 Qd8xd2 22. Rd1xd2 Be5xc3 23. b2xc3 e6-e5 24. f2-f4 Ra8-e8 25. f4xg5 h6xg5 26. Rd2-b2 b7-b6 27. Rb2-b5 Kg8-f8 28. a2-a4 Nf6-d7 29. Bg2-f3 Kf8-e7 30. a4-a5 Ke7-d6 31. a5xb6 Kd6-c6 32. Bf3-e2 a7xb6 33. Rb5-b1 Re8-e7 34. Kg1-f2 f7-f6 1/2-1/2",
    "1. d2-d4 Ng8-f6 2. Ng1-f3 e7-e6 3. c2-c4 b7-b6 4. Nb1-c3 Bf8-b4 5. Qd1-b3 a7-a5 6. g2-g3 Bc8-b7 7. Bf1-g2 O-O 8. O-O d7-d5 9. c4xd5 e6xd5 10. Bc1-f4 Nb8-a6 11. Nf3-e5 Nf6-e4 12. Ra1-c1 c7-c5 13. Nc3xe4 d5xe4 14. a2-a3 g7-g5 15. a3xb4 g5xf4 16. b4xc5 Bb7-d5 17. Qb3xb6 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-c4 Bf8-c5 4. O-O Ng8-f6 5. d2-d3 O-O 6. c2-c3 d7-d6 7. a2-a4 a7-a6 8. Rf1-e1 Bc5-a7 9. Nb1-d2 Nf6-g4 10. Re1-e2 Bc8-d7 11. Nd2-f1 h7-h6 12. Nf1-g3 Bd7-e6 13. Bc4xe6 f7xe6 14. h2-h3 Ng4-f6 15. Bc1-e3 Ba7xe3 16. Re2xe3 Qd8-e8 17. a4-a5 Nf6-d7 18. Qd1-b3 Ra8-b8 19. Qb3-a2 Qe8-f7 20. Ra1-f1 Kg8-h7 21. d3-d4 Rb8-e8 22. d4-d5 e6xd5 23. e4xd5 Nc6-e7 24. c3-c4 g7-g6 25. Ng3-e4 Kh7-g7 26. b2-b4 Ne7-f5 27. Re3-d3 Qf7-e7 28. Rf1-e1 Nd7-f6 29. Qa2-b2 Nf6xe4 30. Re1xe4 Qe7-f6 31. b4-b5 Nf5-d4 32. Nf3xd4 e5xd4 33. Re4xe8 Rf8xe8 34. Qb2xd4 Re8-e1+ 35. Kg1-h2 Qf6xd4 36. Rd3xd4 a6xb5 37. c4xb5 Re1-b1 1/2-1/2",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 a7-a6 6. Bc1-e3 e7-e6 7. Qd1-d2 b7-b5 8. f2-f3 Nb8-d7 9. O-O-O b5-b4 10. Nc3-e2 d6-d5 11. e4xd5 Nf6xd5 12. Be3-f2 Bc8-b7 13. Kc1-b1 Qd8-c7 14. Rd1-c1 Nd7-e5 15. c2-c4 Nd5-b6 16. Ne2-f4 Ne5xc4 17. Bf1xc4 Nb6xc4 18. Rc1xc4 Qc7xc4 19. Rh1-c1 Qc4xc1+ 20. Qd2xc1 Bf8-d6 21. Nf4-h5 Rh8-g8 22. Bf2-g3 Ra8-c8 23. Qc1-g5 Bd6-e7 24. Nh5xg7+ Ke8-d7 25. Qg5-d2 Bb7-d5 26. Ng7-h5 Rc8-c5 27. Nd4-b3 Rc5-b5 28. Nh5-f4 Rg8-c8 29. Qd2-d4 a6-a5 30. Qd4-d3 Rc8-c4 31. Nf4xd5 e6xd5 32. Nb3-d2 Rc4-c5 33. Nd2-e4 Rc5-c4 34. b2-b3 1-0",
    "1. e2-e4 d7-d6 2. d2-d4 Ng8-f6 3. Nb1-c3 g7-g6 4. Bc1-g5 c7-c6 5. Qd1-d2 b7-b5 6. Bf1-d3 h7-h6 7. Bg5xf6 e7xf6 8. Ng1-e2 h6-h5 9. Nc3-d1 Bf8-h6 10. Nd1-e3 O-O 11. O-O Nb8-d7 12. a2-a4 b5xa4 13. Ra1xa4 Nd7-b6 14. Ra4-a5 d6-d5 15. f2-f4 Rf8-e8 16. e4xd5 c6xd5 17. Rf1-a1 a7-a6 18. b2-b3 Ra8-a7 19. Ne3-d1 Re8-e6 20. g2-g3 f6-f5 21. Nd1-b2 Bh6-f8 22. Nb2-a4 Nb6xa4 23. Ra5xa4 h5-h4 24. Kg1-f2 Qd8-f6 25. c2-c3 Qf6-h8 26. Kf2-g2 f7-f6 27. Ne2-g1 Qh8-h5 28. Bd3-e2 Qh5-h6 29. Be2-f3 h4xg3 30. h2xg3 Re6-e4 31. Ra4-a5 g6-g5 32. Bf3xe4 f5xe4 33. f4xg5 Qh6-h5 34. Ra5xd5 e4-e3 35. Qd2xe3 Ra7-e7 36. Qe3-f3 Bc8-g4 37. Qf3-f4 Re7-h7 38. Rd5-d8 Qh5-h2+ 39. Kg2-f1 Bg4-h3+ 40. Ng1xh3 Qh2xh3+ 41. Kf1-e2 Rh7-e7+ 42. Ke2-d3 Qh3-g2 43. Qf4-f1 Qg2xg3+ 44. Kd3-c2 Qg3-h2+ 45. Kc2-c1 Re7-e2 46. Ra1xa6 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 d7-d6 8. c2-c3 O-O 9. h2-h3 Nc6-b8 10. d2-d4 Nb8-d7 11. Nb1-d2 Bc8-b7 12. Bb3-c2 Rf8-e8 13. Nd2-f1 Be7-f8 14. Nf1-g3 g7-g6 15. a2-a4 c7-c5 16. d4-d5 c5-c4 17. Nf3-h2 Bf8-g7 18. Nh2-g4 Nf6xg4 19. Qd1xg4 Qd8-c7 20. Bc1-e3 Nd7-c5 21. Qg4-e2 Ra8-b8 22. a4xb5 a6xb5 23. Ra1-a7 Rb8-a8 24. Re1-a1 Ra8xa7 25. Ra1xa7 Bg7-f6 26. Ng3-f1 Qc7-b8 27. Ra7-a1 Bb7-c8 28. Nf1-d2 Bc8-d7 29. Kg1-h1 Qb8-c7 30. Qe2-f1 Bf6-g7 31. f2-f4 e5xf4 32. Qf1xf4 Nc5-a4 33. Nd2-f3 Re8-b8 34. Ra1-a2 Bd7-e8 35. Qf4-g3 Qc7-e7 36. Be3-g5 Qe7-c7 37. Bg5-f4 Rb8-b6 38. Bf4-e3 Rb6-a6 39. Qg3-e1 Qc7-b7 40. Be3-d4 Bg7xd4 41. Nf3xd4 Be8-d7 42. Bc2xa4 b5xa4 43. Qe1-e2 Ra6-b6 44. Kh1-h2 Qb7-c8 45. Nd4-c2 Rb6-b8 46. Nc2-b4 Qc8-e8 47. Qe2xc4 Qe8-e5+ 48. Kh2-h1 Rb8-e8 49. Nb4-c6 Qe5xe4 50. Ra2xa4 Qe4-b1+ 51. Kh1-h2 Qb1xb2 52. Ra4-a2 Qb2-b6 53. Qc4-b4 Qb6-c5 54. Qb4xc5 d6xc5 55. Ra2-a5 Bd7xc6 56. d5xc6 Re8-c8 57. Ra5xc5 Kg8-f8 58. c6-c7 1/2-1/2",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 O-O 8. a2-a4 Bc8-b7 9. d2-d3 Rf8-e8 10. Nb1-c3 b5-b4 11. Nc3-d5 Nc6-a5 12. Nd5xe7+ Qd8xe7 13. Bb3-a2 d7-d5 14. e4xd5 Qe7-d6 15. Bc1-d2 Bb7xd5 16. Ba2xd5 Nf6xd5 17. Qd1-e2 c7-c5 18. Nf3-g5 Qd6-g6 19. Qe2-f3 Ra8-d8 20. h2-h4 Na5-c6 21. h4-h5 Qg6-f6 22. Qf3-e4 g7-g6 23. Qe4-c4 Nc6-d4 24. Ng5-e4 Qf6-f5 25. Ne4-g3 Qf5-d7 26. Bd2-g5 Rd8-c8 27. Ng3-e4 Rc8-c6 28. Ne4xc5 Rc6xc5 29. Qc4xc5 Nd4-e6 30. Qc5-c4 Ne6xg5 31. Ra1-c1 Nd5-f4 32. Re1-e3 Qd7-g4 33. Qc4-c6 Re8-c8 34. Qc6-b7 e5-e4 35. g2-g3 Ng5-f3+ 36. Re3xf3 e4xf3 37. h5xg6 Nf4-e2+ 38. Kg1-f1 Qg4-h3+ 0-1",
    "1. e2-e4 Nb8-c6 2. Ng1-f3 d7-d6 3. d2-d4 Ng8-f6 4. Nb1-c3 Bc8-g4 5. Bf1-b5 a7-a6 6. Bb5xc6+ b7xc6 7. h2-h3 Bg4-h5 8. Qd1-e2 e7-e6 9. Bc1-f4 Bf8-e7 10. O-O-O Qd8-b8 11. g2-g4 Bh5-g6 12. h3-h4 h7-h5 13. g4-g5 Nf6-d7 14. Nf3-e1 c6-c5 15. d4-d5 e6-e5 16. Bf4-e3 Qb8-b7 17. Kc1-b1 Ra8-b8 18. Be3-c1 O-O 19. f2-f4 e5xf4 20. Ne1-d3 f4-f3 21. Qe2xf3 Nd7-e5 22. Nd3xe5 d6xe5 23. Nc3-e2 f7-f5 24. g5xf6 Rf8xf6 25. Qf3-g2 Qb7-b4 26. Ne2-g3 Rf6-f4 27. Rh1-g1 Rf4-g4 28. Qg2-e2 Qb4-b5 29. Qe2-f3 Rb8-f8 30. Ng3-f5 Bg6xf5 31. e4xf5 e5-e4 32. Qf3-e3 Be7-f6 33. Rg1xg4 h5xg4 34. Qe3xe4 c5-c4 35. b2-b3 g4-g3 36. Qe4-g4 Rf8-e8 37. Qg4xg3 Qb5-a5 38. Bc1-b2 c4-c3 39. Bb2-c1 Re8-e5 40. Qg3-g6 Qa5-c5 41. h4-h5 Qc5-f2 42. h5-h6 Qf2xf5 43. Qg6-g2 Qf5-e4 44. Qg2-h3 Qe4-f5 45. Qh3-h2 Qf5-h5 46. Qh2xh5 Re5xh5 47. Bc1-f4 Bf6-d8 48. h6xg7 Kg8xg7 49. a2-a3 a6-a5 50. Rd1-d3 Rh5-f5 51. Bf4-g3 Rf5-f1+ 52. Kb1-a2 Bd8-f6 53. Bg3xc7 Rf1-f2 54. Rd3-g3+ Kg7-f7 55. Bc7xa5 Bf6-e5 56. Rg3xc3 Be5xc3 57. Ba5xc3 Rf2xc2+ 58. Bc3-b2 Kf7-e7 59. b3-b4 Ke7-d6 60. Ka2-b3 Rc2-c8 61. a3-a4 Kd6xd5 62. b4-b5 Kd5-c5 63. Bb2-a3+ Kc5-b6 64. Kb3-b4 Rc8-h8 65. a4-a5+ Kb6-a7 1/2-1/2",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-c3 Bf8-b4 4. e4-e5 Qd8-d7 5. Bc1-d2 b7-b6 6. h2-h4 h7-h5 7. Nc3-e2 Bb4-e7 8. Ne2-f4 g7-g6 9. Bf1-d3 Bc8-a6 10. Nf4xg6 f7xg6 11. Bd3xg6+ Ke8-d8 12. Bg6xh5 Qd7-b5 13. Ng1-h3 Rh8xh5 14. Qd1xh5 Qb5xb2 15. Ra1-d1 Qb2xd4 16. c2-c3 Qd4-e4+ 17. Bd2-e3 Kd8-d7 18. Nh3-f4 Ba6-c4 19. Qh5-g4 Qe4xe5 20. Nf4xd5 Bc4xd5 21. Qg4xg8 Qe5xc3+ 22. Ke1-f1 Qc3-c2 23. Rd1xd5+ e6xd5 24. Qg8xd5+ Be7-d6 25. Rh1-h3 Nb8-c6 26. g2-g4 Ra8-f8 27. g4-g5 Nc6-e7 28. Qd5-b5+ c7-c6 29. Qb5-e2 Qc2-f5 30. Kf1-g2 Ne7-d5 31. h4-h5 Qf5-e4+ 32. Kg2-f1 Nd5-c3 33. Qe2-a6 Qe4-g4 34. Rh3-h1 Qg4-d1+ 0-1",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4xd5 c6xd5 4. c2-c4 Ng8-f6 5. Nb1-c3 e7-e6 6. Ng1-f3 Bf8-b4 7. c4xd5 e6xd5 8. Nf3-e5 O-O 9. Bf1-d3 Nb8-c6 10. Ne5xc6 b7xc6 11. O-O Bc8-g4 12. Qd1-c2 Bg4-h5 13. Nc3-e2 Qd8-b6 14. Ne2-f4 Bh5-g6 15. Nf4xg6 h7xg6 16. Bc1-e3 Nf6-g4 17. Qc2-e2 Ng4xe3 18. f2xe3 Bb4-d6 19. Rf1-f3 c6-c5 20. Qe2-f2 c5-c4 21. Bd3-f1 Bd6-e7 22. g2-g4 Ra8-b8 23. b2-b3 Qb6-e6 24. b3xc4 d5xc4 25. Ra1-c1 Rb8-c8 26. h2-h3 c4-c3 27. Bf1-d3 Be7-g5 28. Bd3-c2 Qe6xa2 29. h3-h4 Bg5-e7 30. h4-h5 Qa2-e6 31. Qf2-g2 Be7-g5 32. Rc1-e1 a7-a5 33. h5xg6 f7xg6 34. Bc2-e4 Rf8xf3 35. Qg2xf3 Rc8-f8 36. Qf3-g2 Kg8-h7 37. Be4-d5 Qe6-e7 38. Bd5-e4 Bg5-h6 39. Qg2-c2 Qe7-h4 40. Be4xg6+ Kh7-h8 0-1",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Nb8-d7 5. Ng1-f3 Ng8-f6 6. Ne4-g3 e7-e6 7. Bf1-d3 Bf8-e7 8. O-O O-O 9. Qd1-e2 c6-c5 10. Rf1-d1 Qd8-c7 11. c2-c4 Rf8-e8 12. b2-b3 b7-b6 13. Bc1-b2 Bc8-b7 14. d4xc5 b6xc5 15. Nf3-e5 Nd7-f8 16. Ng3-e4 h7-h6 17. Ne4xf6+ Be7xf6 18. Bd3-e4 Bb7xe4 19. Qe2xe4 Re8-d8 20. g2-g3 Ra8-b8 21. Rd1-d3 Rd8xd3 22. Ne5xd3 Bf6xb2 23. Nd3xb2 f7-f5 24. Qe4-e3 e6-e5 25. Ra1-d1 Nf8-e6 26. Rd1-d5 Ne6-d4 27. Nb2-a4 Rb8-d8 28. Rd5xd8+ Qc7xd8 29. Na4xc5 Qd8-a8 30. Nc5-d7 Nd4-f3+ 31. Kg1-f1 Nf3xh2+ 32. Kf1-e2 Qa8-h1 33. Nd7xe5 Qh1-f1+ 34. Ke2-d2 Qf1-a1 35. Qe3-c3 Qa1xa2+ 36. Qc3-c2 Qa2-a5+ 37. Qc2-c3 Qa5xe5 0-1",
    "1. d2-d4 Ng8-f6 2. Ng1-f3 e7-e6 3. b2-b3 c7-c5 4. e2-e3 b7-b6 5. Bc1-b2 Bc8-b7 6. Bf1-d3 Bf8-e7 7. Nb1-d2 d7-d6 8. O-O O-O 9. Rf1-e1 Nb8-d7 10. c2-c4 Rf8-e8 11. e3-e4 c5xd4 12. Bb2xd4 Qd8-c7 13. Ra1-c1 Ra8-d8 14. Bd3-b1 Nd7-e5 15. b3-b4 Qc7-b8 16. Qd1-b3 Nf6-d7 17. Bd4-b2 Be7-f6 18. Nf3-d4 Bf6-g5 19. Rc1-d1 Rd8-c8 20. Nd2-f3 Bg5-f6 21. Rd1-c1 Bb7-a6 22. Nf3xe5 Nd7xe5 23. Nd4-b5 d6-d5 24. e4xd5 e6xd5 25. Qb3-a3 Ba6xb5 26. c4xb5 Rc8xc1 27. Re1xc1 Ne5-c4 28. Qa3-h3 Bf6xb2 29. Qh3xh7+ Kg8-f8 30. Qh7-h8+ Kf8-e7 31. Rc1-e1+ Bb2-e5 32. Qh8xg7 Ke7-d8 0-1",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Nb1-c3 Bf8-b4 4. f2-f3 Nf6-h5 5. Ng1-h3 f7-f5 6. Bc1-g5 Bb4-e7 7. f3-f4 Be7xg5 8. f4xg5 g7-g6 9. e2-e4 Nh5-g7 10. e4xf5 g6xf5 11. Bf1-e2 O-O 12. Qd1-d2 d7-d6 13. O-O-O Qd8-e7 14. g2-g4 Nb8-c6 15. g5-g6 h7xg6 16. Qd2-h6 Ng7-e8 17. g4xf5 Qe7-g7 18. Qh6-h4 e6-e5 19. f5xg6 Nc6xd4 20. Rd1xd4 Bc8xh3 21. Rd4-d3 Bh3-e6 22. Be2-g4 Be6xg4 23. Qh4xg4 Qg7-h6+ 24. Kc1-b1 Ne8-f6 25. Qg4-e6+ Kg8-g7 26. Qe6-e7+ Kg7-g8 27. g6-g7 Rf8-f7 28. Qe7-e6 Qh6-h7 29. Qe6-h3 Qh7xh3 30. Rd3xh3 Rf7xg7 31. Rh1-f1 Ra8-f8 32. Rh3-f3 Nf6-h7 33. Rf3xf8+ Nh7xf8 34. Nc3-d5 Nf8-h7 35. Nd5-e3 Rg7-f7 36. Rf1-g1+ Kg8-f8 37. Rg1-g2 Rf7-f3 38. Ne3-d5 c7-c6 39. Nd5-c3 Nh7-f6 40. Rg2-e2 b7-b6 41. Kb1-c2 Kf8-f7 42. b2-b4 Kf7-e6 43. b4-b5 c6-c5 44. Re2-g2 e5-e4 45. Rg2-g7 Rf3-f2+ 46. Kc2-d1 Ke6-e5 47. Nc3-e2 d6-d5 48. c4xd5 Nf6xd5 49. Rg7-g5+ Ke5-d6 50. Rg5-g6+ Kd6-e7 51. Rg6-g5 Ke7-e6 52. h2-h4 e4-e3 53. h4-h5 Rf2-f1+ 54. Kd1-c2 Nd5-b4+ 55. Kc2-c3 Rf1-f2 56. a2-a3 Nb4-d5+ 57. Kc3-d3 c5-c4+ 58. Kd3-e4 Rf2xe2 59. Rg5-e5+ Ke6-f6 60. Re5xd5 Re2-e1 61. Rd5-d6+ Kf6-g5 62. Rd6-c6 Kg5xh5 63. Rc6xc4 1/2-1/2",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Ng1-f3 Bf8-b4+ 4. Nb1-d2 b7-b6 5. e2-e3 Bc8-b7 6. Bf1-d3 O-O 7. O-O d7-d5 8. a2-a3 Bb4-e7 9. b2-b4 Nb8-d7 10. c4xd5 e6xd5 11. Ra1-b1 a7-a6 12. Nf3-e5 Be7-d6 13. f2-f4 Nf6-e4 14. Nd2xe4 d5xe4 15. Ne5xd7 Qd8xd7 16. Bd3-c4 Bd6-e7 17. Bc1-d2 Bb7-d5 18. Rb1-c1 c7-c6 19. Qd1-b3 Rf8-c8 20. Rc1-c2 b6-b5 21. Bc4xd5 c6xd5 22. Rc2xc8+ Ra8xc8 23. Rf1-a1 Rc8-c4 24. a3-a4 g7-g6 25. h2-h3 Kg8-g7 26. a4xb5 a6xb5 27. Ra1-a6 Be7-h4 28. Kg1-h2 Qd7-c8 29. Ra6-a7 Qc8-c6 30. Qb3-d1 Rc4-c2 31. Qd1-g4 Bh4-f6 32. Qg4-d1 Rc2-b2 33. Bd2-e1 Qc6-c4 34. Qd1-g4 Qc4-f1 35. f4-f5 g6-g5 36. Qg4-g3 h7-h5 37. Be1-c3 Rb2-c2 0-1",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 g7-g6 6. Bf1-e2 Bf8-g7 7. O-O O-O 8. Bc1-g5 Nb8-c6 9. Nd4-b3 a7-a6 10. a2-a4 Bc8-e6 11. Kg1-h1 Nc6-a5 12. Nb3xa5 Qd8xa5 13. Qd1-d2 Rf8-d8 14. Rf1-d1 Kg8-f8 15. f2-f4 Ra8-c8 16. Be2-f3 Rd8-d7 17. Bg5-h4 Rd7-c7 18. Qd2-e2 Qa5-b4 19. Bh4-e1 Qb4-c4 20. Qe2-f2 Nf6-g4 21. Bf3xg4 Be6xg4 22. Rd1-d3 Qc4-c5 23. Qf2-g3 Bg4-e6 24. a4-a5 Qc5-h5 25. Be1-d2 Rc7-c5 26. Qg3-f2 Rc5xa5 27. Ra1-e1 Qh5-c5 28. Qf2-h4 Qc5-h5 29. Qh4-f2 Qh5-c5 30. Qf2-h4 Qc5-h5 31. Qh4-f2 1/2-1/2",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 Ng8-f6 4. e4-e5 Nf6-d7 5. c2-c3 c7-c5 6. f2-f4 Nb8-c6 7. Nd2-f3 Qd8-b6 8. g2-g3 Bf8-e7 9. Bf1-h3 c5xd4 10. c3xd4 O-O 11. Ke1-f1 f7-f5 12. Ng1-e2 a7-a5 13. g3-g4 Qb6-d8 14. Rh1-g1 Nd7-b6 15. a2-a4 Bc8-d7 16. b2-b3 Ra8-c8 17. g4xf5 e6xf5 18. Bc1-b2 Bd7-e8 19. Ne2-g3 Be8-g6 20. Ra1-c1 Nb6-a8 21. Kf1-f2 Na8-c7 22. Qd1-d2 Nc7-e6 23. Bh3-f1 Qd8-b6 24. Qd2-e3 Nc6-b4 25. Bf1-b5 Nb4-c2 26. Qe3-d2 Be7-b4 27. Qd2-d1 Ne6xf4 28. Bb5-f1 Nf4-e6 29. Rc1xc2 Rc8xc2+ 30. Qd1xc2 f5-f4 31. Bf1-d3 f4xg3+ 32. Rg1xg3 Ne6xd4 33. Bb2xd4 Qb6xd4+ 34. Kf2-e2 Bg6xd3+ 35. Qc2xd3 Qd4-b2+ 36. Ke2-f1 Qb2xe5 37. Kf1-g2 Qe5-e4 38. Qd3xe4 d5xe4 39. Nf3-g5 Bb4-d6 40. Rg3-h3 Rf8-f4 41. Ng5-e6 Rf4-f6 42. Ne6-d8 b7-b6 43. Rh3-e3 Bd6-c7 44. Nd8-b7 Rf6-e6 0-1",
    "1. e2-e4 c7-c5 2. Ng1-f3 Nb8-c6 3. d2-d4 c5xd4 4. Nf3xd4 e7-e5 5. Nd4-b5 d7-d6 6. a2-a4 Bc8-e6 7. Nb1-c3 a7-a6 8. Nb5-a3 Ng8-f6 9. Bc1-g5 Ra8-c8 10. Bf1-c4 Bf8-e7 11. O-O O-O 12. Bg5xf6 Be7xf6 13. Nc3-d5 Bf6-g5 14. c2-c3 Nc6-e7 15. Qd1-d3 Ne7-g6 16. Rf1-d1 Kg8-h8 17. Bc4-a2 f7-f5 18. e4xf5 Be6xf5 19. Qd3-e2 Bg5-h4 20. f2-f3 Qd8-g5 21. Ba2-b1 e5-e4 22. Bb1xe4 Bf5xe4 23. Qe2xe4 Rc8-e8 24. Qe4-g4 Qg5xg4 25. f3xg4 Re8-e2 26. Na3-c4 Rf8-f2 27. Nd5-e3 Ng6-f4 28. Rd1xd6 Nf4xg2 29. g4-g5 Bh4xg5 30. Ne3-g4 Rf2-f4 31. Ra1-f1 h7-h5 32. Ng4-e5 Kh8-h7 33. Ne5-f3 Bg5-e7 34. Rd6-d2 Be7-c5+ 35. Kg1-h1 Re2xd2 36. Nc4xd2 Ng2-e3 37. Nf3-g5+ Kh7-g6 38. Rf1xf4 Kg6xg5 39. Rf4-f7 Kg5-h4 40. Nd2-e4 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 a7-a6 6. Bc1-e3 e7-e5 7. Nd4-b3 Bc8-e6 8. Qd1-d2 Bf8-e7 9. f2-f3 O-O 10. O-O-O b7-b5 11. g2-g4 b5-b4 12. Nc3-a4 Nb8-c6 13. g4-g5 Nf6-d7 14. h2-h4 Nc6-a5 15. Kc1-b1 Na5-c4 16. Bf1xc4 Be6xc4 17. Qd2xb4 Bc4-e2 18. Rd1-g1 Be2xf3 19. Rh1-h2 d6-d5 20. Qb4-b7 d5-d4 21. Be3-c1 Ra8-b8 22. Qb7-c6 Qd8-c8 23. Qc6xc8 Rf8xc8 24. Nb3-d2 Bf3xe4 25. Nd2xe4 Rc8-c4 26. b2-b4 Rc4xb4+ 27. Na4-b2 Nd7-c5 28. c2-c3 Rb4-b6 29. Ne4xc5 Be7xc5 30. c3xd4 Bc5xd4 31. Rg1-d1 f7-f5 32. g5xf6 g7xf6 33. a2-a3 f6-f5 34. Kb1-a2 f5-f4 35. Nb2-c4 Rb6-c6 36. Rh2-g2+ Kg8-f8 37. Nc4-a5 Rc6-b6 38. Na5-c4 Rb6-c6 39. Nc4-a5 Rc6-b6 40. Rg2-c2 Rb6-b1 41. Rd1xd4 e5xd4 42. Bc1xf4 Rb8-b5 43. a3-a4 Rb5xa5 44. Ka2xb1 Ra5xa4 45. Rc2-f2 Kf8-g7 46. Kb1-b2 Kg7-g6 47. Bf4-g5 Ra4-b4+ 48. Kb2-c2 Rb4-b6 49. Kc2-d3 Rb6-d6 50. Rf2-g2 Kg6-h5 51. Bg5-e7 Rd6-d7 52. Rg2-e2 a6-a5 53. Re2-e5+ Kh5-g4 54. Be7-f6 Rd7-d6 55. Re5-e4+ Kg4-h5 56. Bf6xd4 a5-a4 57. Kd3-c2 Rd6-c6+ 58. Bd4-c3 Rc6-a6 59. Kc2-b2 Ra6-a8 60. Kb2-a3 1-0",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Nb8-d7 5. Ne4-g5 Nd7-b6 6. Ng1-f3 g7-g6 7. Bf1-d3 Bf8-g7 8. O-O Ng8-h6 9. Rf1-e1 O-O 10. c2-c3 Qd8-c7 11. Nf3-e5 Nb6-d5 12. Qd1-f3 Nh6-f5 13. Qf3-h3 h7-h5 14. Qh3-f3 Qc7-d8 15. h2-h3 Qd8-e8 16. Ne5-c4 b7-b5 17. Nc4-a5 b5-b4 18. Bc1-d2 Ra8-b8 19. c3-c4 Nd5-b6 20. Bd3xf5 Bc8xf5 21. Na5xc6 Rb8-b7 22. b2-b3 Rb7-c7 23. Nc6xb4 a7-a5 24. Nb4-d3 Bg7xd4 25. Ra1-d1 Qe8-a8 26. Bd2-f4 Rc7-d7 27. c4-c5 Nb6-d5 28. Ng5-e4 Nd5-c3 29. Ne4xc3 Qa8xf3 30. g2xf3 Bd4xc3 31. c5-c6 Rd7xd3 32. Rd1xd3 Bc3xe1 33. Rd3-d1 Be1-b4 34. c6-c7 Rf8-c8 35. Rd1-c1 f7-f6 36. Bf4-e3 Bf5xh3 37. Be3-b6 Kg8-f7 38. Rc1-c4 Bh3-e6 39. Rc4-c2 h5-h4 40. a2-a3 Bb4xa3 41. Bb6xa5 Ba3-d6 42. b3-b4 Rc8-a8 0-1",
    "1. e2-e4 c7-c6 2. Nb1-c3 d7-d5 3. Ng1-f3 Bc8-g4 4. d2-d4 d5xe4 5. Nc3xe4 Bg4xf3 6. Qd1xf3 Qd8xd4 7. Bc1-d2 Ng8-f6 8. Bf1-d3 Nb8-d7 9. Bd2-c3 Qd4-d5 10. O-O O-O-O 11. Ra1-d1 e7-e6 12. Qf3-e3 Nf6xe4 13. Bd3xe4 Qd5-c5 14. Qe3-f4 f7-f5 15. Be4-f3 Rh8-g8 16. Qf4-a4 e6-e5 17. Qa4-b3 Bf8-e7 18. Qb3-e6 Rg8-f8 19. b2-b4 Qc5xc3 20. Qe6xe7 Rf8-g8 21. Qe7-e6 Rg8-e8 22. Qe6xf5 Re8-e7 23. Qf5xh7 Qc3xb4 24. Qh7-h3 Kc8-c7 25. Bf3-g4 Nd7-f6 26. Rd1xd8 Kc7xd8 27. Rf1-d1+ Kd8-c7 28. Qh3-d3 Re7-e8 29. Bg4-f5 Qb4-c5 30. h2-h3 a7-a5 31. c2-c4 Kc7-b6 32. Bf5-g6 Re8-e7 33. Qd3-d8+ Kb6-a7 34. Rd1-d6 Nf6-g8 35. Rd6-d1 Ng8-h6 36. Rd1-d6 Re7-f7 37. Rd6-d2 Rf7-f6 38. Bg6-e4 Nh6-f7 39. Qd8-d3 Nf7-d6 40. Rd2-c2 Rf6-f4 41. Be4-h7 Rf4xc4 42. Rc2xc4 Nd6xc4 43. Bh7-g8 Nc4-d6 44. Qd3-g6 Qc5-c1+ 45. Kg1-h2 Qc1-f4+ 46. Kh2-g1 Nd6-e4 47. f2-f3 Qf4-e3+ 48. Kg1-h2 Ne4-c3 49. Qg6xg7 Nc3-e2 50. h3-h4 Qe3-g1+ 0-1",
    "1. e2-e4 c7-c6 2. c2-c4 d7-d5 3. e4xd5 c6xd5 4. c4xd5 Ng8-f6 5. Nb1-c3 Nf6xd5 6. d2-d4 Nb8-c6 7. Ng1-f3 e7-e6 8. Bf1-d3 Bf8-e7 9. O-O O-O 10. Bc1-e3 Nd5xe3 11. f2xe3 g7-g6 12. Qd1-e2 Be7-f6 13. a2-a3 Bf6-g7 14. Ra1-d1 Bc8-d7 15. Kg1-h1 Ra8-c8 16. Bd3-b1 Qd8-a5 17. Qe2-f2 Nc6-e7 18. e3-e4 Qa5-b6 19. Rd1-d2 Ne7-c6 20. Nc3-a4 Qb6-b5 21. Na4-c3 Qb5-a5 22. e4-e5 f7-f6 23. e5xf6 Rf8xf6 24. Nc3-e4 Rf6-f4 25. g2-g3 Rf4-f8 26. Qf2-g2 b7-b6 27. Rf1-d1 Nc6-e7 28. Ne4-c3 Ne7-d5 29. Nc3xd5 Qa5xd5 30. Nf3-e5 Qd5xg2+ 31. Kh1xg2 Bd7-a4 32. Rd1-e1 Ba4-b3 33. Bb1-e4 Rc8-c7 34. Be4-f3 Rf8-d8 35. Ne5-c6 Rd8-d6 36. Nc6-e5 Bg7-h6 37. Rd2-d3 Bb3-a4 0-1",
    "1. e2-e4 c7-c6 2. d2-d3 d7-d5 3. Nb1-d2 g7-g6 4. g2-g3 Bf8-g7 5. Bf1-g2 e7-e5 6. Ng1-f3 Ng8-e7 7. O-O O-O 8. Rf1-e1 Nb8-d7 9. Nd2-f1 h7-h6 10. g3-g4 Qd8-c7 11. Nf1-g3 d5xe4 12. d3xe4 Nd7-c5 13. Bc1-e3 Nc5-e6 14. c2-c3 Ne6-f4 15. h2-h3 Bc8-e6 16. Qd1-c2 b7-b6 17. b2-b3 Kg8-h8 18. Ra1-d1 Nf4xg2 19. Kg1xg2 f7-f5 20. e4xf5 g6xf5 21. g4xf5 Ne7xf5 22. c3-c4 Qc7-f7 23. Nf3-g1 b6-b5 24. c4xb5 Be6-d5+ 25. Kg2-h2 Nf5xe3 0-1",
    "1. Ng1-f3 Ng8-f6 2. c2-c4 e7-e6 3. Nb1-c3 Bf8-b4 4. g2-g3 b7-b6 5. Bf1-g2 Bc8-b7 6. O-O O-O 7. Qd1-c2 a7-a5 8. b2-b3 Rf8-e8 9. a2-a3 Bb4xc3 10. Qc2xc3 d7-d6 11. d2-d4 Nb8-d7 12. Bc1-b2 Qd8-e7 13. Rf1-e1 Bb7-e4 14. Nf3-d2 Be4xg2 15. Kg1xg2 e6-e5 16. e2-e4 c7-c6 17. Ra1-d1 Qe7-f8 18. f2-f4 e5xd4 19. Qc3xd4 Nd7-c5 20. h2-h3 h7-h6 21. Bb2-a1 Re8-e6 22. b3-b4 a5xb4 23. a3xb4 Nc5-d7 24. Qd4-d3 Ra8-a2 25. Ba1-c3 d6-d5 26. c4xd5 c6xd5 27. e4-e5 Nf6-e4 28. Rd1-a1 Ne4xc3 29. Qd3xc3 Ra2xa1 30. Re1xa1 Re6-e8 31. Ra1-a7 Nd7-b8 32. Nd2-f3 Re8-c8 33. Ra7-c7 Rc8-d8 34. b4-b5 Nb8-d7 35. Qc3-c6 Nd7-c5 36. Qc6xb6 Nc5-e6 37. Rc7-c2 d5-d4 38. f4-f5 d4-d3 39. Rc2-c1 Ne6-g5 40. Nf3xg5 h6xg5 41. e5-e6 f7xe6 42. Qb6xe6+ Kg8-h8 43. Rc1-f1 d3-d2 44. Rf1-d1 Rd8-d3 45. Qe6-e4 1/2-1/2",
    "1. e2-e4 d7-d6 2. d2-d4 Ng8-f6 3. Nb1-c3 g7-g6 4. Bc1-g5 Bf8-g7 5. Qd1-d2 O-O 6. f2-f4 c7-c6 7. e4-e5 Nf6-d5 8. Nc3xd5 c6xd5 9. O-O-O Nb8-c6 10. Ng1-e2 Bc8-e6 11. h2-h3 Ra8-c8 12. g2-g4 d6xe5 13. f4xe5 f7-f6 14. e5xf6 e7xf6 15. Bg5-e3 Qd8-b6 16. Kc1-b1 Rf8-e8 17. Ne2-f4 Be6-f7 18. Bf1-g2 Re8-d8 19. h3-h4 Nc6-a5 20. Bg2-f1 Na5-c4 21. Bf1xc4 d5xc4 22. c2-c3 Qb6-c6 23. Kb1-a1 Qc6-f3 24. g4-g5 Rd8-e8 25. Rh1-h3 Qf3-g4 26. Rd1-g1 Qg4-f5 27. g5xf6 Bg7-h6 28. Rh3-f3 Qf5-e4 29. Qd2-f2 Rc8-c6 30. Nf4xg6 h7xg6 31. Be3xh6 Rc6-e6 32. Rf3-e3 Qe4-d5 33. Re3xe6 Re8xe6 34. a2-a3 b7-b5 35. Qf2-f4 Qd5-h5 36. Bh6-g5 Qh5-e2 37. d4-d5 Re6-e5 38. h4-h5 Re5xd5 39. h5xg6 Rd5-d1+ 40. Ka1-a2 Rd1xg1 41. Qf4-b8+ 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 Nb8-c6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 d7-d6 6. Bc1-e3 e7-e5 7. Nd4-f3 Bf8-e7 8. h2-h3 O-O 9. Bf1-c4 Bc8-e6 10. Bc4-b3 b7-b5 11. Qd1-d3 Nc6-b4 12. Qd3-e2 Qd8-d7 13. O-O Rf8-c8 14. Rf1-d1 Qd7-b7 15. Be3-g5 Rc8-c6 16. Bb3xe6 f7xe6 17. a2-a3 Nb4-a6 18. Nc3xb5 Ra8-d8 19. Ra1-b1 Na6-c5 20. Nb5-c3 Rd8-c8 21. Nf3-e1 Rc6-b6 22. Bg5xf6 Be7xf6 23. b2-b4 Nc5-d7 24. Rb1-b3 Rb6-c6 25. Nc3-a4 Bf6-e7 26. Na4-b2 Nd7-b6 27. Qe2-g4 d6-d5 28. Nb2-d3 Nb6-c4 29. Ne1-f3 Be7-f6 30. Nd3-c5 Qb7-f7 31. Nf3-g5 h7-h5 32. Qg4-g3 Qf7-e7 33. h3-h4 Rc8-d8 34. Rb3-b1 Rc6-d6 35. Qg3-h3 Bf6xg5 36. h4xg5 Qe7-f7 37. g5-g6 Qf7xg6 38. Nc5-b7 Qg6xe4 39. Nb7xd8 Rd6xd8 40. Qh3xe6+ Kg8-h7 41. Qe6-f7 Rd8-d6 42. Rb1-b3 Rd6-g6 43. Qf7-f3 Qe4xc2 44. Rb3-c3 Qc2-a2 45. Qf3xh5+ Rg6-h6 46. Qh5-f5+ Kh7-g8 47. Rc3-h3 Rh6xh3 48. Qf5xh3 Nc4-b6 49. Qh3-e6+ Kg8-h8 50. Qe6xe5 Qa2xa3 51. Qe5-e8+ Kh8-h7 52. Qe8-h5+ Kh7-g8 53. Rd1-e1 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Ba4xc6 d7xc6 7. d2-d3 Nf6-d7 8. Nb1-d2 f7-f6 9. Rf1-e1 Nd7-f8 10. Nd2-f1 g7-g5 11. Bc1-e3 c6-c5 12. c2-c3 Nf8-g6 13. Qd1-a4+ b7-b5 14. Qa4-c2 f6-f5 15. e4xf5 Bc8xf5 16. Nf1-g3 Bf5xd3 17. Qc2-d2 O-O 18. Be3xg5 Rf8xf3 19. Bg5xe7 Qd8xe7 20. g2xf3 c5-c4 21. f3-f4 Qe7-f6 22. f4-f5 Ng6-h4 23. Qd2-e3 Kg8-h8 24. a2-a4 Ra8-e8 25. a4xb5 a6xb5 26. f2-f3 Bd3xf5 27. Kg1-f2 Bf5-d3 28. Ng3-h5 Qf6-c6 29. Re1-g1 Bd3-g6 30. Rg1-g5 Nh4-f5 31. Qe3-d2 Re8-f8 32. Qd2-e2 Qc6-c5+ 33. Kf2-e1 Nf5-e3 34. Rg5xg6 h7xg6 35. Nh5-g3 Ne3-f5 36. Ng3xf5 Rf8xf5 37. Ra1-a8+ Kh8-g7 38. Ra8-c8 e5-e4 39. f3-f4 Rf5xf4 0-1",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 d7-d6 8. c2-c3 O-O 9. h2-h3 Nc6-a5 10. Bb3-c2 c7-c5 11. d2-d4 Na5-c6 12. d4-d5 Nc6-a7 13. Nb1-d2 c5-c4 14. Nd2-f1 a6-a5 15. Nf1-e3 Qd8-c7 16. Qd1-e2 Nf6-h5 17. a2-a4 Nh5-f4 18. Qe2-f1 Bc8-d7 19. Ne3-f5 Ra8-e8 20. a4xb5 Bd7xb5 21. Bc1xf4 e5xf4 22. Bc2-a4 Re8-b8 23. Ba4xb5 Rb8xb5 24. Nf5xe7+ Qc7xe7 25. Qf1xc4 Rf8-b8 26. e4-e5 Rb5xb2 27. Qc4xf4 Na7-b5 28. c3-c4 Rb2-b4 29. Ra1xa5 Qe7-c7 30. Ra5xb5 Rb8xb5 31. e5xd6 Qc7-d8 32. d6-d7 1-0",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Nb8-d7 5. Ng1-f3 Ng8-f6 6. Ne4xf6+ Nd7xf6 7. g2-g3 Bc8-g4 8. Bf1-g2 e7-e6 9. O-O Bf8-d6 10. b2-b3 O-O 11. Bc1-b2 Qd8-a5 12. h2-h3 Bg4-h5 13. Qd1-c1 Ra8-d8 14. Nf3-e5 Qa5-c7 15. Rf1-e1 Bh5-g6 16. Ne5xg6 h7xg6 17. c2-c4 Rd8-d7 18. Qc1-e3 Rf8-d8 19. Ra1-c1 Qc7-a5 20. a2-a3 Bd6-f8 21. b3-b4 Qa5-b6 22. Rc1-b1 Qb6-c7 23. Bb2-a1 a7-a5 24. Qe3-b3 Rd8-a8 25. Ba1-c3 a5xb4 26. a3xb4 Rd7-d8 27. Bc3-b2 Rd8-b8 28. b4-b5 Qc7-d7 29. b5xc6 b7xc6 30. Qb3-f3 Ra8-a2 31. Bb2-a1 Rb8xb1 32. Re1xb1 Qd7-a7 33. Qf3-d1 Qa7-a6 34. Qd1-f1 Kg8-h7 35. Ba1-c3 Ra2-c2 36. Rb1-a1 Qa6-b6 37. Ra1-b1 Qb6-a6 1/2-1/2",
    "1. c2-c4 Ng8-f6 2. Ng1-f3 c7-c5 3. g2-g3 b7-b6 4. Bf1-g2 Bc8-b7 5. O-O e7-e6 6. b2-b3 Bf8-e7 7. Bc1-b2 O-O 8. d2-d3 d7-d6 9. e2-e4 Nb8-d7 10. Qd1-e2 a7-a6 11. Nb1-c3 Qd8-c7 12. Nf3-d2 Nf6-e8 13. f2-f4 Be7-f6 14. Nd2-f3 Ra8-b8 15. Kg1-h1 Bb7-a8 16. a2-a4 Qc7-d8 17. Ra1-d1 Qd8-c7 18. Rd1-c1 Rb8-c8 19. Rf1-d1 Qc7-a7 20. Bb2-a1 Ne8-c7 21. g3-g4 Rf8-e8 22. g4-g5 Bf6-e7 23. d3-d4 c5xd4 24. Nf3xd4 e6-e5 25. Nd4-f5 Nc7-e6 26. Rd1-f1 Be7-f8 27. Qe2-g4 Kg8-h8 28. f4xe5 Nd7xe5 29. Qg4-g3 Ne6-c5 30. Rc1-b1 Qa7-b8 31. Nc3-d5 b6-b5 32. c4xb5 a6xb5 33. a4-a5 b5-b4 34. Qg3-f4 Qb8-b5 35. Nf5-h6 Ba8xd5 36. Ba1xe5 d6xe5 37. Nh6xf7+ Bd5xf7 38. Qf4xf7 Qb5-b7 39. Qf7-h5 g7-g6 40. Qh5-g4 Rc8-a8 0-1",
    "1. d2-d4 Ng8-f6 2. Ng1-f3 e7-e6 3. g2-g3 b7-b6 4. Bf1-g2 Bc8-b7 5. O-O c7-c5 6. c2-c4 c5xd4 7. Nf3xd4 Bb7xg2 8. Kg1xg2 1/2-1/2",
    "1. e2-e4 c7-c6 2. c2-c4 d7-d5 3. e4xd5 c6xd5 4. c4xd5 Ng8-f6 5. Nb1-c3 Nf6xd5 6. d2-d4 Nb8-c6 7. Ng1-f3 Bc8-g4 8. Qd1-b3 Bg4xf3 9. g2xf3 e7-e6 10. Qb3xb7 Nc6xd4 11. Bf1-b5+ Nd4xb5 12. Qb7-c6+ Ke8-e7 13. Qc6xb5 Qd8-d7 14. Nc3xd5+ Qd7xd5 15. Qb5xd5 e6xd5 16. Bc1-e3 Ke7-e6 17. Rh1-g1 Bf8-d6 18. Rg1xg7 Bd6-e5 19. Rg7-g4 Be5xb2 20. Ra1-b1 Rh8-b8 21. Ke1-f1 Bb2-e5 22. Rb1-e1 Ke6-d6 23. Be3-c1 Rb8-e8 24. Bc1-a3+ Kd6-d7 25. Rg4-g5 f7-f6 26. Rg5-g7+ Kd7-c6 27. Re1-c1+ Kc6-b6 28. Ba3-c5+ Kb6-a6 29. f3-f4 Be5xf4 30. Rc1-c3 Re8-c8 31. Rg7xh7 Bf4-e5 32. Rc3-a3+ Ka6-b5 33. Bc5xa7 Rc8-c7 34. Ra3-b3+ Kb5-c4 35. Rh7xc7+ 1/2-1/2",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Nb1-c3 Bf8-b4 4. Qd1-c2 c7-c5 5. d4xc5 Nb8-c6 6. Ng1-f3 Bb4xc5 7. Bc1-g5 h7-h6 8. Bg5-h4 a7-a6 9. e2-e3 b7-b6 10. Ra1-d1 Bc8-b7 11. a2-a3 Bc5-e7 12. Bf1-e2 O-O 13. O-O d7-d6 14. Rd1-d2 Qd8-b8 15. Rf1-d1 Rf8-d8 16. b2-b4 Nf6-e8 17. Bh4-g3 Nc6-e5 18. Nf3xe5 d6xe5 19. Rd2xd8 Be7xd8 20. Nc3-a4 Bb7-c6 21. c4-c5 b6-b5 22. Na4-c3 Bd8-c7 23. f2-f3 Ne8-f6 24. e3-e4 Nf6-d7 25. Bg3-h4 f7-f6 26. Bh4-f2 Kg8-f7 27. Qc2-b3 Qb8-b7 28. Rd1-b1 Nd7-b8 29. a3-a4 b5xa4 30. Nc3xa4 Bc6-b5 31. Be2-c4 Bb5xc4 32. Qb3xc4 Nb8-c6 33. Na4-c3 Kf7-e7 34. h2-h4 Ra8-b8 35. b4-b5 a6xb5 36. Nc3xb5 Qb7-a6 37. Qc4-f1 Bc7-a5 38. Kg1-h2 Ba5-b4 39. Nb5-d4 Qa6-a8 40. Nd4xc6+ Qa8xc6 41. Qf1-c4 1-0",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Nb8-d7 5. Ng1-e2 Ng8-f6 6. Ne2-g3 g7-g6 7. Bf1-c4 Bf8-g7 8. c2-c3 O-O 9. O-O Qd8-c7 10. Rf1-e1 b7-b6 11. Ne4xf6+ Nd7xf6 12. Qd1-f3 Bc8-b7 13. Bc1-f4 Qc7-d7 14. Qf3-e2 e7-e6 15. Ra1-d1 Rf8-d8 16. Bf4-e5 c6-c5 17. Bc4-a6 Qd7-c6 18. Ba6xb7 Qc6xb7 19. d4xc5 b6xc5 20. c3-c4 h7-h5 21. h2-h3 a7-a5 22. Qe2-e3 Qb7-c6 23. Qe3-c3 Rd8xd1 24. Re1xd1 Nf6-e8 25. Be5xg7 Ne8xg7 26. Qc3-e5 Qc6-a4 27. Rd1-c1 Qa4xa2 28. Ng3-e4 Ng7-e8 29. Ne4xc5 a5-a4 30. Nc5-e4 Qa2-b3 31. c4-c5 Qb3-b5 32. Rc1-d1 Qb5-c6 33. Ne4-d6 Ra8-d8 1/2-1/2",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4xd5 c6xd5 4. c2-c4 Ng8-f6 5. Nb1-c3 e7-e6 6. Ng1-f3 Bf8-b4 7. Bf1-d3 d5xc4 8. Bd3xc4 O-O 9. O-O Nb8-d7 10. Bc1-g5 a7-a6 11. a2-a4 b7-b6 12. Qd1-b3 Bb4xc3 13. b2xc3 Qd8-c7 14. Ra1-b1 Bc8-b7 15. Bg5xf6 Bb7xf3 16. Bf6-e5 Nd7xe5 17. d4xe5 Bf3-e4 18. Rb1-e1 Qc7-c6 19. f2-f3 Be4-g6 20. Kg1-h1 Rf8-c8 21. Bc4-e2 a6-a5 22. Be2-b5 Qc6xc3 23. Qb3xc3 Rc8xc3 24. Re1-c1 Rc3-c5 25. Rf1-d1 h7-h5 26. Rc1xc5 b6xc5 27. Kh1-g1 Ra8-c8 28. Rd1-c1 Rc8-c7 29. Kg1-f2 Kg8-f8 30. Kf2-e3 Kf8-e7 31. Ke3-f4 f7-f6 32. h2-h4 Bg6-e8 33. Rc1-c4 Be8-d7 34. g2-g4 Bd7-c8 35. g4xh5 Bc8-b7 36. Rc4-c1 Rc7-c8 37. h5-h6 g7xh6 38. Rc1-c2 Bb7-d5 39. Bb5-c4 f6xe5+ 40. Kf4xe5 Rc8-f8 41. Bc4xd5 Rf8-f5+ 42. Ke5-e4 Rf5xd5 43. Rc2-b2 Ke7-f6 44. Ke4-e3 Kf6-e5 45. Rb2-b6 c5-c4 46. f3-f4+ Ke5-f5 47. Rb6-c6 Rd5-d3+ 48. Ke3-e2 Rd3-d4 49. Rc6-c5+ Kf5xf4 50. Rc5xa5 c4-c3 51. Ra5-c5 Rd4xa4 52. Rc5xc3 Ra4-a2+ 53. Ke2-f1 e6-e5 54. Rc3-c4+ e5-e4 55. Rc4-c5 Ra2-h2 56. Rc5-h5 Kf4-f3 57. Rh5-f5+ Kf3-g3 58. Rf5-h5 Rh2xh4 59. Rh5-e5 Kg3-f3 60. Re5-f5+ Kf3-e3 61. Rf5-d5 Rh4-f4+ 62. Kf1-g1 Ke3-e2 63. Rd5-h5 Rf4-g4+ 64. Kg1-h2 Rg4-g6 65. Rh5-a5 e4-e3 66. Ra5-a1 Ke2-f2 67. Ra1-a8 Rg6-f6 0-1",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Ng1-f3 Bf8-b4+ 4. Bc1-d2 Qd8-e7 5. g2-g3 Nb8-c6 6. Bf1-g2 Bb4xd2+ 7. Nb1xd2 O-O 8. O-O d7-d6 9. e2-e4 a7-a5 10. d4-d5 Nc6-b8 11. Nf3-e1 Nb8-a6 12. Ne1-d3 e6-e5 13. a2-a3 c7-c6 14. d5xc6 b7xc6 15. b2-b4 c6-c5 16. b4-b5 Na6-c7 17. f2-f4 Nf6-d7 18. f4-f5 Nd7-b6 19. a3-a4 Nc7-e8 20. Nd3-b2 f7-f6 21. Nd2-b3 g7-g6 22. Qd1-d2 Qe7-a7 23. g3-g4 Kg8-g7 24. Rf1-f3 Rf8-h8 25. Ra1-f1 h7-h6 26. h2-h4 Qa7-c7 27. Rf3-g3 Ra8-a7 28. Nb3xa5 Nb6xa4 29. Nb2xa4 Qc7xa5 30. Qd2xa5 Ra7xa5 31. Na4-b6 Bc8-b7 32. h4-h5 g6-g5 33. Rf1-d1 Kg7-f8 34. Rd1-d3 Kf8-e7 35. Rd3-a3 Ra5xa3 36. Rg3xa3 Ke7-d8 37. Kg1-f2 Kd8-c7 38. Nb6-d5+ Kc7-b8 39. Bg2-f1 Rh8-h7 40. b5-b6 Bb7-c6 41. Kf2-e3 Rh7-b7 42. Ra3-a6 Rb7-g7 43. Bf1-d3 Bc6-b7 44. Ra6-a1 Bb7-c6 45. Ke3-d2 Rg7-b7 46. Ra1-a6 Rb7-g7 47. Kd2-c3 Bc6-b7 48. Ra6-a1 Bb7-c6 49. Bd3-c2 Rg7-b7 50. Bc2-a4 Bc6xa4 51. Ra1xa4 Rb7-d7 52. Kc3-b3 Rd7-f7 53. Ra4-a6 Rf7-g7 54. Kb3-a4 Rg7-h7 55. Ka4-b5 Rh7-f7 56. Kb5-c6 Rf7-g7 57. Ra6-a7 1-0",
    "1. e2-e4 d7-d5 2. e4xd5 Ng8-f6 3. c2-c4 c7-c6 4. Nb1-c3 c6xd5 5. d2-d4 g7-g6 6. Qd1-b3 Bf8-g7 7. c4xd5 O-O 8. Ng1-e2 Nb8-d7 9. Ne2-f4 Nd7-b6 10. Bf1-e2 Bc8-g4 11. Be2xg4 Nf6xg4 12. O-O Bg7xd4 13. h2-h3 Ng4-f6 14. Rf1-d1 Bd4-e5 15. a2-a4 Qd8-c7 16. a4-a5 Nb6-c4 17. Nc3-b5 Qc7-c8 18. Nf4-d3 a7-a6 19. Nd3xe5 a6xb5 20. Ne5xc4 b5xc4 21. Qb3-f3 Rf8-d8 22. d5-d6 Qc8-e6 23. Qf3xb7 e7xd6 24. Qb7-f3 Nf6-e4 25. Bc1-e3 d6-d5 26. Be3-d4 Rd8-e8 27. Qf3-f4 Ne4-d6 28. Bd4-c3 Nd6-b5 29. Rd1-e1 Qe6-d6 30. Bc3-e5 Qd6-c6 31. h3-h4 Re8-e6 32. h4-h5 g6xh5 33. Re1-e3 Re6-g6 34. Re3-f3 f7-f6 35. Be5-d4 Rg6-g4 36. Rf3-g3 Rg4xg3 37. Qf4xg3+ Kg8-f7 38. Bd4-b6 Ra8-g8 39. Qg3-f3 Rg8-g5 40. Bb6-e3 Nb5-c7 41. Be3xg5 1-0",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 Ng8-f6 4. e4-e5 Nf6-d7 5. c2-c3 c7-c5 6. f2-f4 Nb8-c6 7. Nd2-f3 Qd8-b6 8. g2-g3 c5xd4 9. c3xd4 Bf8-b4+ 10. Ke1-f2 g7-g5 11. f4xg5 Nd7xe5 12. Nf3xe5 Nc6xe5 13. Bc1-e3 Ne5-c6 14. Ng1-f3 Bb4-f8 15. Qd1-d2 Bf8-g7 16. Kf2-g2 Bc8-d7 17. Ra1-c1 Ra8-d8 18. Be3-f2 O-O 19. Bf1-d3 f7-f5 20. Rh1-e1 Rd8-e8 21. Bd3-b1 Re8-e7 22. Rc1-c3 Bd7-e8 23. Rc3-b3 Qb6-d8 24. Qd2-f4 Be8-h5 25. Qf4-h4 Qd8-e8 26. h2-h3 Nc6-a5 27. Rb3-c3 Na5-c4 28. g3-g4 Bh5-g6 29. g4xf5 Bg6xf5 30. Bb1xf5 Rf8xf5 31. b2-b3 Nc4-d6 32. Re1-c1 Qe8-g6 33. Bf2-g3 Nd6-e4 34. Rc3-c8+ Re7-e8 35. Qh4-g4 Rf5-f8 36. Rc8-c7 Rf8-f7 37. Rc7xf7 Qg6xf7 38. h3-h4 Qf7-d7 39. Nf3-e5 Qd7-d6 40. Qg4-h5 Re8-f8 41. Rc1-c8 Rf8xc8 42. Qh5-f7+ Kg8-h8 43. Ne5-g6+ h7xg6 44. Bg3xd6 Rc8-c2+ 45. Kg2-h3 Ne4xd6 46. Qf7xe6 Nd6-f5 47. Qe6xg6 Nf5xd4 48. h4-h5 Rc2-c3+ 49. Kh3-g4 Nd4-f3 50. Qg6-e8+ Kh8-h7 51. h5-h6 Nf3-h2+ 52. Kg4-h4 Bg7-d4 53. Qe8-f7+ Kh7-h8 54. g5-g6 1-0",
    "1. e2-e4 d7-d6 2. d2-d4 Ng8-f6 3. Nb1-c3 g7-g6 4. Bc1-g5 Nb8-d7 5. f2-f4 h7-h6 6. Bg5-h4 Nf6-h5 7. Ng1-e2 g6-g5 8. f4xg5 e7-e6 9. Bh4-f2 h6xg5 10. Qd1-d2 a7-a6 11. Bf2-e3 Bf8-e7 12. O-O-O c7-c5 13. d4xc5 d6xc5 14. g2-g4 Nh5-f6 15. Rh1-g1 Nd7-e5 16. Qd2xd8+ Be7xd8 17. Be3xg5 Bd8-e7 18. Bg5-f4 Ne5-g6 19. Bf4-c7 Nf6-d7 20. Kc1-b1 b7-b5 21. Ne2-f4 Ng6-e5 22. Nf4-d3 Ne5-g6 23. Bf1-g2 Bc8-b7 24. h2-h3 Ra8-c8 25. Bc7-g3 Ng6-h4 26. Bg2-h1 Nh4-g6 27. Nd3-f2 Nd7-e5 28. Nc3-e2 Be7-g5 29. b2-b3 Bg5-h4 30. Nf2-d3 Bh4xg3 31. Rg1xg3 Ke8-e7 32. Ne2-f4 c5-c4 33. Nf4xg6+ Ne5xg6 34. Nd3-b2 c4xb3 35. a2xb3 Ng6-f4 36. e4-e5 Bb7xh1 37. Rd1xh1 Rc8-c5 38. Rg3-f3 Nf4-g6 39. Nb2-d3 Rc5-d5 40. Rh1-f1 Ng6xe5 41. Nd3xe5 Rd5xe5 42. Rf3xf7+ Ke7-d6 43. Rf1-f3 Kd6-c6 44. Kb1-b2 Re5-e2 45. Rf3-c3+ Kc6-b6 46. Rf7-c7 b5-b4 47. Rc3-c6+ Kb6-b5 48. Rc6-c4 a6-a5 49. Rc7-c5+ Kb5-b6 50. Rc5-h5 Rh8-a8 51. h3-h4 e6-e5 52. Rh5-h6+ Kb6-b7 53. Rh6-e6 Ra8-c8 54. Re6-e7+ Kb7-b8 55. Rc4xc8+ Kb8xc8 56. h4-h5 Kc8-d8 57. Re7-a7 Re2-h2 58. Ra7xa5 Rh2-h4 59. Ra5-d5+ Kd8-c7 60. Rd5xe5 Rh4xg4 61. Re5-c5+ Kc7-b6 62. Rc5-c4 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 g7-g6 4. d2-d4 Nc6xd4 5. Nf3xd4 e5xd4 6. Qd1xd4 Qd8-f6 7. Qd4-d3 a7-a6 8. Bb5-c4 Bf8-g7 9. Nb1-c3 Ng8-e7 10. O-O O-O 11. Qd3-g3 Qf6-b6 12. Bc4-b3 Qb6-a5 13. Qg3-h4 Rf8-e8 14. Bc1-g5 Qa5-e5 15. f2-f4 Qe5-c5+ 16. Kg1-h1 d7-d5 17. Ra1-e1 Bc8-d7 18. Bg5xe7 Re8xe7 19. Nc3xd5 Re7-e8 20. f4-f5 Kg8-h8 21. Nd5-f6 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 d7-d6 8. c2-c3 O-O 9. h2-h3 Nc6-a5 10. Bb3-c2 c7-c5 11. d2-d4 Bc8-b7 12. Nb1-d2 c5xd4 13. c3xd4 Ra8-c8 14. d4-d5 Nf6-h5 15. Nd2-b3 Na5-c4 16. a2-a4 Qd8-c7 17. Bc2-b1 g7-g6 18. Nb3-d2 Rc8-b8 19. b2-b3 Nc4xd2 20. Bc1xd2 Bb7-c8 21. a4xb5 a6xb5 22. Qd1-c1 Qc7-b7 23. Bd2-e3 Rb8-a8 24. Ra1xa8 Qb7xa8 25. Qc1-c7 Rf8-e8 26. Bb1-d3 Qa8-a6 27. Re1-c1 Nh5-f6 28. Be3-g5 Kg8-g7 29. Rc1-c6 Qa6-a1+ 30. Kg1-h2 Qa1-d1 31. Bg5xf6+ Kg7xf6 32. Bd3xb5 Re8-d8 33. Rc6-c3 g6-g5 34. Nf3xg5 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 d7-d6 8. c2-c3 O-O 9. h2-h3 Nc6-a5 10. Bb3-c2 c7-c5 11. d2-d4 Qd8-c7 12. Nb1-d2 Bc8-d7 13. Nd2-f1 c5xd4 14. c3xd4 Ra8-c8 15. Nf1-e3 Na5-c6 16. d4-d5 Nc6-b4 17. Bc2-b1 a6-a5 18. a2-a3 Nb4-a6 19. b2-b4 g7-g6 20. Bc1-d2 a5xb4 21. a3xb4 Qc7-b7 22. Nf3-h2 Na6-c7 1/2-1/2",
    "1. d2-d4 Ng8-f6 2. Ng1-f3 e7-e6 3. c2-c4 b7-b6 4. a2-a3 Bc8-a6 5. Nb1-d2 Ba6-b7 6. Qd1-c2 c7-c5 7. e2-e4 c5xd4 8. e4-e5 Nf6-g4 9. h2-h3 Ng4-h6 10. Nd2-e4 Nh6-f5 11. g2-g4 Nf5-h4 12. Nf3xh4 Qd8xh4 13. Bc1-g5 Bb7xe4 14. Qc2xe4 Qh4xg5 15. h3-h4 Qg5-d8 16. Qe4xa8 Qd8-c7 17. Qa8-e4 Nb8-c6 18. f2-f4 Bf8-e7 19. b2-b4 h7-h5 20. g4-g5 g7-g6 21. Bf1-g2 O-O 22. Rh1-h3 Rf8-c8 23. Ra1-c1 Be7-f8 24. Rh3-d3 a7-a5 25. b4-b5 Nc6-e7 26. Bg2-h3 d7-d5 27. e5xd6 Qc7xd6 28. Qe4xd4 Qd6-c7 29. Qd4-e4 Ne7-f5 30. Bh3xf5 e6xf5 31. Qe4-e5 Qc7-b7 32. Ke1-f2 Qb7-a8 33. Kf2-g3 Rc8-e8 34. Qe5-d5 Qa8-b8 35. Rc1-f1 Bf8-c5 36. a3-a4 Kg8-g7 37. Qd5-c6 Re8-e6 38. Qc6-d7 Qb8-a8 39. Rd3-d5 Bc5-b4 40. Kg3-f2 Qa8-b8 41. Kf2-g3 Bb4-d6 42. Rd5-d3 Qb8-a8 43. Rd3-d5 Qa8-b8 44. Rd5-d4 Qb8-a8 45. Rf1-f3 Qa8-b8 46. Rf3-d3 Bd6-c7 47. Kg3-f2 Bc7-e5 48. Rd4-d5 Be5-b2 49. Kf2-g3 Re6-e4 50. Qd7-d6 Qb8-e8 51. Qd6xb6 Re4-e2 52. Rd5-d7 Re2-c2 53. Qb6-e3 Qe8-a8 54. Rd3-d2 Rc2-c3 55. Rd7-d3 Rc3xc4 56. Rd2xb2 Qa8-h1 57. Rd3-d8 Kg7-h7 58. Rb2-h2 Qh1-a1 59. Rh2-d2 Rc4-e4 60. Qe3-f2 Re4-e1 61. Kg3-g2 1-0",
    "1. Ng1-f3 Ng8-f6 2. c2-c4 b7-b6 3. g2-g3 c7-c5 4. Bf1-g2 Bc8-b7 5. O-O e7-e6 6. Nb1-c3 Bf8-e7 7. d2-d4 c5xd4 8. Qd1xd4 O-O 9. Rf1-d1 d7-d6 10. Bc1-g5 Nf6-e8 11. Bg5xe7 Qd8xe7 12. Qd4-h4 Qe7xh4 13. Nf3xh4 Bb7xg2 14. Nh4xg2 Nb8-c6 15. Nc3-b5 Nc6-a5 16. Ng2-e3 Na5-b7 17. Rd1-d3 a7-a6 18. Rd3-a3 f7-f5 19. Ra1-d1 Kg8-f7 20. Ne3-c2 Nb7-a5 21. Nb5xd6+ Ne8xd6 22. Rd1xd6 Na5xc4 23. Rd6-d7+ Kf7-e8 24. Rd7xg7 Nc4xa3 25. Nc2xa3 Rf8-f7 26. Rg7-g8+ Rf7-f8 27. Rg8-g7 Ra8-c8 28. Rg7-a7 Rf8-f7 29. Ra7xa6 Rf7-b7 30. b2-b3 Ke8-e7 31. Na3-c4 b6-b5 32. Nc4-e5 Ke7-f6 33. Ne5-d3 Rb7-c7 34. b3-b4 Rc7-c6 35. Ra6-a5 Rc6-c2 36. Kg1-f1 Rc8-d8 37. Ra5-a6 Kf6-e7 38. h2-h4 Rd8-d6 39. Ra6-a7+ Rd6-d7 40. Ra7-a5 Ke7-f6 41. f2-f3 h7-h6 42. Ra5-a6 Kf6-e7 43. Nd3-c5 Rd7-d1+ 44. Kf1-f2 Rd1-d6 45. Ra6-a7+ Ke7-e8 46. Ra7-a3 Ke8-f7 47. Nc5-d3 Kf7-f6 48. Kf2-e3 Rd6-c6 49. Ra3-a5 Rc6-c3 50. Ra5xb5 Rc2xa2 51. Rb5-b6 Rc3-a3 52. g3-g4 Ra2-c2 53. g4xf5 Kf6xf5 54. Rb6-b8 Kf5-f6 55. Rb8-g8 Ra3-a2 56. Rg8-g2 Kf6-f5 57. h4-h5 e6-e5 58. b4-b5 Rc2-c4 59. b5-b6 Ra2-a8 60. Rg2-g6 e5-e4 61. f3xe4+ Rc4xe4+ 62. Ke3-f3 Ra8-e8 63. Rg6xh6 Re4xe2 64. Nd3-f2 Re8-e3+ 65. Kf3-g2 Re3-b3 1/2-1/2",
    "1. c2-c4 Ng8-f6 2. g2-g3 e7-e5 3. Bf1-g2 Nb8-c6 4. Nb1-c3 Bf8-b4 5. e2-e4 Bb4xc3 6. d2xc3 d7-d6 7. Qd1-e2 O-O 8. Ng1-f3 a7-a5 9. O-O Bc8-e6 10. Nf3-e1 Qd8-d7 11. Ne1-c2 Be6-h3 12. Bc1-g5 Bh3xg2 13. Kg1xg2 Qd7-e6 14. Bg5xf6 Qe6xf6 15. Nc2-e3 Nc6-e7 16. Ra1-d1 Qf6-g6 1/2-1/2",
    "1. Ng1-f3 Ng8-f6 2. c2-c4 b7-b6 3. g2-g3 c7-c5 4. Bf1-g2 Bc8-b7 5. O-O g7-g6 6. b2-b3 Bf8-g7 7. Bc1-b2 O-O 8. Nb1-c3 Nb8-a6 9. Ra1-c1 d7-d5 10. Nc3xd5 Nf6xd5 11. Bb2xg7 Kg8xg7 12. c4xd5 Qd8xd5 13. d2-d4 Rf8-d8 14. d4xc5 Na6xc5 15. Qd1-c2 Qd5-f5 16. Qc2xf5 g6xf5 17. Rf1-d1 Nc5-e6 18. Nf3-e5 Bb7xg2 19. Kg1xg2 Rd8xd1 20. Rc1xd1 Ra8-d8 21. Rd1xd8 Ne6xd8 22. Kg2-f3 f7-f6 23. Ne5-d3 Nd8-c6 24. e2-e4 f5xe4+ 25. Kf3xe4 e7-e6 26. g3-g4 Kg7-g6 27. f2-f4 h7-h5 28. h2-h3 h5xg4 29. h3xg4 Kg6-f7 30. a2-a4 a7-a6 31. Nd3-c1 b6-b5 32. a4xb5 a6xb5 33. Nc1-e2 Nc6-a5 34. Ne2-c1 Na5-c6 35. Nc1-d3 Kf7-g6 36. Ke4-e3 Kg6-f7 37. Nd3-c5 e6-e5 38. f4-f5 Nc6-d4 39. Ke3-e4 Kf7-e7 40. Ke4-d3 Ke7-d6 41. Nc5-e4+ Kd6-e7 42. Kd3-c3 Nd4-c6 43. Kc3-b2 Nc6-d4 44. Kb2-c3 1/2-1/2",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4-e5 Bc8-f5 4. Ng1-e2 e7-e6 5. Ne2-g3 Bf5-g6 6. h2-h4 h7-h5 7. Bf1-d3 Bg6xd3 8. c2xd3 Qd8-b6 9. Ng3-e2 Ng8-e7 10. Nb1-d2 Ne7-f5 11. Nd2-f3 c6-c5 12. d4xc5 Bf8xc5 13. d3-d4 Bc5-e7 14. Bc1-g5 Be7-b4+ 15. Ke1-f1 Nb8-c6 16. g2-g3 Bb4-e7 17. Bg5xe7 Nf5xe7 18. Qd1-d2 O-O-O 19. Ra1-c1 Kc8-b8 20. Kf1-g2 Rd8-c8 21. Rc1-c5 Ne7-f5 22. Rh1-c1 Qb6-d8 23. b2-b4 Qd8-d7 24. Ne2-f4 Nc6-e7 25. Nf3-g5 Rc8xc5 26. d4xc5 Rh8-f8 27. Nf4xh5 Ne7-g6 28. Ng5-f3 d5-d4 29. Nh5-f4 Qd7-c6 30. Nf4xg6 f7xg6 31. Qd2-e2 g6-g5 32. h4xg5 d4-d3 33. Qe2xd3 Nf5-d4 34. Qd3xd4 Qc6xf3+ 35. Kg2-f1 Qf3-h1+ 36. Kf1-e2 Qh1-f3+ 1/2-1/2",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 d7-d6 8. c2-c3 O-O 9. h2-h3 Nf6-d7 10. d2-d4 Nd7-b6 11. Nb1-d2 Be7-f6 12. Nd2-f1 Rf8-e8 13. Nf1-g3 g7-g6 14. Bc1-h6 Nc6-a5 15. Bb3-c2 c7-c5 16. d4-d5 Bc8-d7 17. Qd1-d2 Qd8-e7 18. Re1-f1 Bf6-g7 19. Bh6xg7 Kg8xg7 20. Nf3-h2 Qe7-h4 21. b2-b3 Qh4-f4 22. Qd2-e1 Qf4-g5 23. Kg1-h1 Na5-b7 24. Ng3-e2 Re8-f8 25. f2-f4 e5xf4 26. Ne2xf4 Ra8-e8 27. Qe1-d2 Kg7-g8 28. Ra1-e1 f7-f6 29. Rf1-f3 Nb7-d8 30. Qd2-f2 Nd8-f7 31. b3-b4 Nf7-e5 32. Rf3-g3 Qg5-h4 33. b4xc5 d6xc5 34. Nf4-d3 Nb6-c4 35. Nd3xc5 Bd7-c8 36. Nh2-f3 Qh4-h6 37. Nf3-d4 Ne5-g4 38. Qf2-g1 Qh6-h4 39. Rg3-f3 f6-f5 40. Nd4-e6 f5xe4 41. Bc2xe4 Ng4-f2+ 42. Rf3xf2 Rf8xf2 43. Be4-d3 Nc4-d6 44. Re1-d1 Re8-f8 45. Ne6xf8 Bc8xh3 46. Qg1-h2 Rf2xg2 47. Qh2xd6 Rg2xa2 0-1",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4xd5 c6xd5 4. c2-c4 Ng8-f6 5. Nb1-c3 e7-e6 6. Ng1-f3 Bf8-e7 7. c4xd5 e6xd5 8. Bf1-b5+ Nb8-c6 9. O-O O-O 10. Nf3-e5 Qd8-b6 11. Bc1-g5 Nc6-d8 12. Qd1-d3 Nd8-e6 13. Bg5-e3 Rf8-d8 14. f2-f4 Ne6-c7 15. Bb5-a4 Qb6-a6 16. Qd3xa6 Nc7xa6 17. f4-f5 Kg8-f8 18. Ba4-b3 Na6-c7 19. g2-g4 Nf6-e8 20. g4-g5 Rd8-d6 21. Rf1-f3 a7-a5 22. Ra1-c1 f7-f6 23. g5-g6 h7-h6 24. Ne5-f7 Rd6-b6 25. Rc1-e1 Be7-b4 26. Bb3-a4 Bb4xc3 27. b2xc3 Rb6-b2 28. Nf7xh6 b7-b5 29. Nh6-f7 b5xa4 30. Be3-c1 Rb2xa2 31. Rf3-h3 Nc7-e6 32. f5xe6 Kf8-e7 33. c3-c4 d5xc4 34. d4-d5 Bc8-b7 35. Re1-d1 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 Nb8-c6 6. Bc1-e3 e7-e5 7. Nd4-f3 Bf8-e7 8. Bf1-c4 Bc8-e6 9. Qd1-e2 Ra8-c8 10. Bc4-b3 O-O 11. O-O Nc6-a5 12. Rf1-d1 Na5-c4 13. Be3-c1 Qd8-c7 14. h2-h3 a7-a6 15. Nf3-h2 Rf8-d8 16. Nh2-g4 Nf6xg4 17. h3xg4 Rd8-d7 18. Nc3-d5 Be6xd5 19. Rd1xd5 h7-h6 20. c2-c3 Qc7-c6 21. a2-a4 Rd7-c7 22. a4-a5 b7-b6 23. Bb3xc4 Qc6xc4 24. Qe2xc4 Rc7xc4 25. a5xb6 Rc4xe4 26. Bc1-e3 Rc8-b8 27. Ra1xa6 g7-g6 28. Rd5-a5 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Nb8-c6 5. Nb1-c3 a7-a6 6. Nd4xc6 b7xc6 7. Bf1-d3 d7-d5 8. O-O Ng8-f6 9. Rf1-e1 Bc8-b7 10. e4-e5 Nf6-d7 11. Nc3-a4 Qd8-c7 12. Bc1-f4 c6-c5 13. c2-c4 d5xc4 14. Bd3-e4 Bf8-e7 15. Ra1-c1 O-O 16. Rc1xc4 Bb7xe4 17. Rc4xe4 Rf8-d8 18. Qd1-c2 Nd7-b6 19. Na4xb6 Qc7xb6 20. Bf4-e3 Qb6-a5 21. Re1-a1 Rd8-d5 22. h2-h3 Ra8-d8 23. Ra1-c1 h7-h6 24. b2-b3 Qa5-b5 25. f2-f4 Rd5-d3 26. Kg1-h2 Qb5-b7 27. Qc2-c4 Rd3-d1 28. Rc1xd1 Rd8xd1 29. Be3-f2 Qb7-b5 1/2-1/2",
    "1. e2-e4 c7-c6 2. c2-c4 d7-d5 3. e4xd5 c6xd5 4. c4xd5 Ng8-f6 5. Bf1-b5+ Nb8-d7 6. Nb1-c3 a7-a6 7. Qd1-a4 Ra8-b8 8. Bb5xd7+ Qd8xd7 9. Qa4xd7+ Bc8xd7 10. d2-d4 b7-b5 11. a2-a3 a6-a5 12. Bc1-f4 Rb8-c8 13. Bf4-d2 b5-b4 14. a3xb4 a5xb4 15. Nc3-a2 Rc8-a8 16. Bd2xb4 Nf6xd5 17. Bb4-a3 e7-e6 18. Ng1-e2 Bf8xa3 19. Na2-c3 Nd5xc3 20. Ne2xc3 Ke8-e7 21. Ra1xa3 1/2-1/2",
    "1. Ng1-f3 Ng8-f6 2. g2-g3 d7-d5 3. Bf1-g2 c7-c6 4. O-O Bc8-g4 5. d2-d3 Nb8-d7 6. Nb1-d2 e7-e5 7. h2-h3 Bg4-h5 8. Qd1-e1 Bf8-c5 9. e2-e4 O-O 10. Nf3-h4 Rf8-e8 11. Nd2-b3 Bc5-f8 12. g3-g4 Bh5-g6 13. g4-g5 Nf6-h5 14. Nh4xg6 h7xg6 15. e4xd5 c6xd5 16. Bg2xd5 Nh5-f4 17. Bc1xf4 e5xf4 18. Qe1-a5 Qd8xg5+ 19. Kg1-h2 Re8-e5 20. c2-c4 Bf8-d6 21. Rf1-g1 Qg5-h4 22. Rg1xg6 f4-f3 23. Kh2-g1 Re5-g5+ 0-1",
    "1. e2-e4 e7-e6 2. d2-d4 c7-c5 3. Ng1-f3 c5xd4 4. Nf3xd4 a7-a6 5. Bf1-d3 g7-g6 6. Bc1-e3 Bf8-g7 7. Nb1-c3 Ng8-e7 8. O-O O-O 9. f2-f4 Nb8-c6 10. Nd4xc6 Ne7xc6 11. Qd1-e1 d7-d5 12. Be3-c5 Rf8-e8 13. e4-e5 b7-b6 14. Bc5-f2 Nc6-b4 15. Bf2-h4 Qd8-c7 16. a2-a3 Nb4xd3 17. c2xd3 Bc8-b7 18. d3-d4 Qc7-c4 19. Ra1-d1 a6-a5 20. Rf1-f3 b6-b5 21. Bh4-f6 b5-b4 22. Nc3-a4 Qc4-c2 23. Na4-c5 Bb7-c6 24. Rf3-f2 Qc2-f5 25. h2-h3 Qf5-h5 26. Rf2-f3 Bg7xf6 27. e5xf6 b4xa3 28. b2xa3 Ra8-b8 29. Rd1-c1 Bc6-b5 30. Rf3-b3 Bb5-c4 31. Rb3xb8 Re8xb8 32. Rc1-b1 Rb8-e8 33. Rb1-b7 Qh5-f5 34. Qe1-h4 h7-h5 35. Nc5-d7 Bc4-a6 36. Rb7-a7 Qf5-d3 37. Kg1-h2 Re8-c8 38. Nd7-e5 Qd3xd4 1-0",
    "1. e2-e4 g7-g6 2. d2-d4 Bf8-g7 3. Nb1-c3 d7-d6 4. Ng1-f3 Ng8-f6 5. Bf1-e2 O-O 6. O-O c7-c6 7. Bc1-f4 Nb8-d7 8. Rf1-e1 Qd8-b6 9. Qd1-c1 e7-e5 10. d4xe5 d6xe5 11. Bf4-e3 Qb6-c7 12. a2-a4 Nf6-g4 13. Be3-d2 Nd7-c5 14. h2-h3 Ng4-f6 15. Be2-f1 a7-a5 16. Bf1-c4 Bc8-e6 17. Bc4xe6 Nc5xe6 18. Bd2-e3 Rf8-e8 19. Nf3-d2 Nf6-d7 20. Qc1-b1 Ne6-d4 21. Nc3-e2 Ra8-d8 22. c2-c3 Nd4-e6 23. Nd2-b3 Bg7-f8 24. Qb1-c2 Bf8-e7 25. f2-f3 Be7-g5 26. Be3-f2 c6-c5 27. Re1-d1 c5-c4 28. Nb3-d2 Bg5xd2 29. Rd1xd2 Nd7-c5 30. Rd2-d5 Nc5-d3 31. Ne2-g3 Rd8xd5 32. e4xd5 Ne6-f4 33. Ra1-d1 Nf4xd5 34. b2-b3 Nd5-f4 35. b3xc4 Qc7xc4 36. Ng3-e4 Re8-e6 37. Bf2-e3 f7-f5 38. Ne4-g5 Re6-d6 39. Be3xf4 e5xf4 40. Kg1-h2 h7-h6 41. Qc2-e2 Nd3-e5 42. Qe2xe5 Rd6xd1 43. Qe5-e8+ Kg8-g7 44. Ng5-e6+ Kg7-f6 45. Qe8-h8+ Kf6-e7 46. Qh8-f8+ 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Nb8-c6 5. Nb1-c3 Qd8-c7 6. Bf1-e2 a7-a6 7. O-O Ng8-f6 8. Bc1-e3 Bf8-e7 9. f2-f4 d7-d6 10. Qd1-e1 O-O 11. Qe1-g3 Bc8-d7 12. Kg1-h1 Ra8-c8 13. Ra1-d1 Nf6-e8 14. Nd4-f3 Nc6-a5 15. f4-f5 Na5-c6 16. Be2-d3 Be7-f6 17. Qg3-h3 Nc6-e5 18. Nf3-g5 h7-h6 19. Ng5-f3 Ne5xd3 20. c2xd3 Bf6xc3 21. b2xc3 Qc7xc3 22. Be3-d4 Qc3-c7 23. g2-g4 e6-e5 24. Bd4-e3 f7-f6 25. Nf3-h4 Bd7-c6 26. Nh4-g6 d6-d5 27. Ng6xf8 Kg8xf8 28. d3-d4 Rc8-d8 29. Kh1-g1 Bc6-b5 30. Rf1-e1 Qc7-c3 31. Qh3-g3 d5xe4 32. Rd1-c1 Qc3-b2 33. d4xe5 f6xe5 34. Rc1-c5 Ne8-f6 35. Rc5xe5 Rd8-d3 36. Re5-e6 Qb2-c3 37. Qg3-b8+ Kf8-f7 38. Be3-f2 Rd3-d7 39. Qb8-f4 Bb5-c6 40. g4-g5 h6xg5 41. Qf4xg5 Rd7-d2 42. Re1-f1 Bc6-d5 43. Bf2-e3 Rd2xa2 44. Re6-d6 Ra2-a1 45. Be3-b6 Ra1xf1+ 46. Kg1xf1 Bd5-c4+ 0-1",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 O-O 8. d2-d4 d7-d6 9. c2-c3 Bc8-g4 10. d4-d5 Nc6-a5 11. Bb3-c2 c7-c6 12. h2-h3 Bg4-c8 13. d5xc6 Qd8-c7 14. a2-a4 Qc7xc6 15. Nb1-d2 Bc8-e6 16. Nf3-g5 Be6-d7 17. a4xb5 a6xb5 18. b2-b4 Na5-b7 19. Ra1xa8 Rf8xa8 20. Bc2-b3 Nb7-d8 21. Qd1-c2 Qc6-b7 22. Qc2-d3 Bd7-c6 23. Bb3-c2 Nf6-h5 24. Nd2-b3 Qb7-c7 25. Nb3-a5 Bc6-d7 26. Ng5-f3 Ra8-c8 27. Bc1-d2 Nd8-e6 28. Bc2-b3 Ne6-f4 29. Qd3-f1 Qc7-b8 30. Re1-d1 Rc8-c7 31. Kg1-h1 Qb8-a8 32. Rd1-e1 g7-g6 33. Nf3-g1 Be7-g5 34. Ng1-f3 Bg5-d8 35. Kh1-h2 Rc7-c8 36. c3-c4 Bd8xa5 37. b4xa5 Bd7-e6 38. Bd2-b4 b5xc4 39. Bb3-c2 d6-d5 40. g2-g4 Nh5-f6 41. Bb4-d6 d5xe4 42. Bd6xe5 e4xf3 43. Be5xf4 Qa8xa5 44. Bf4-e5 Qa5-d2 45. Bc2-d1 Nf6-d7 46. Be5-a1 Qd2-f4+ 47. Kh2-g1 h7-h5 48. g4xh5 Rc8-c5 49. Re1xe6 Rc5-g5+ 50. Kg1-h1 f7xe6 0-1",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 Ng8-f6 4. O-O Bf8-c5 5. c2-c3 O-O 6. d2-d4 Bc5-b6 7. Rf1-e1 d7-d6 8. Bc1-g5 h7-h6 9. Bg5-h4 Bc8-d7 10. Nb1-a3 Rf8-e8 11. d4xe5 Nc6xe5 12. Nf3xe5 Bd7xb5 13. Ne5xf7 Kg8xf7 14. Na3xb5 g7-g5 15. Bh4-g3 Qd8-d7 16. Qd1-d3 h6-h5 17. h2-h3 h5-h4 18. Bg3-h2 g5-g4 19. Nb5-d4 g4-g3 20. f2xg3 c7-c5 21. Re1-f1 c5xd4 22. e4-e5 1-0",
    "1. e2-e4 d7-d6 2. d2-d4 Ng8-f6 3. Nb1-c3 Nb8-d7 4. Ng1-f3 e7-e5 5. Bf1-c4 Bf8-e7 6. O-O O-O 7. Qd1-e2 c7-c6 8. a2-a4 e5xd4 9. Nf3xd4 Rf8-e8 10. Bc1-e3 Be7-f8 11. Nd4-f3 Nf6xe4 12. Nc3xe4 Re8xe4 13. Nf3-g5 Re4-e7 14. Qe2-d3 g7-g6 15. Qd3xd6 Qd8-a5 16. Qd6-f4 Nd7-e5 17. Be3-d4 Bf8-g7 18. Ra1-e1 Qa5-c7 19. Qf4-h4 h7-h6 20. Ng5xf7 Re7xf7 21. Bc4xf7+ Kg8xf7 22. Qh4-f4+ Kf7-e6 23. Re1xe5+ 1-0",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 c7-c5 4. e4xd5 e6xd5 5. Ng1-f3 Nb8-c6 6. Bf1-b5 Bf8-d6 7. O-O Ng8-e7 8. d4xc5 Bd6xc5 9. Nd2-b3 Bc5-d6 10. Bc1-g5 O-O 11. Rf1-e1 Bc8-g4 12. Bg5-h4 Qd8-b6 13. Bb5-d3 h7-h6 14. Bh4-g3 Bd6xg3 15. h2xg3 Rf8-e8 16. Qd1-d2 a7-a5 17. a2-a4 Kg8-f8 18. Bd3-b5 Ra8-d8 19. Nf3-e5 Bg4-e6 20. Bb5xc6 Ne7xc6 21. Ne5xc6 b7xc6 22. Qd2xa5 Qb6xa5 23. Nb3xa5 c6-c5 24. Na5-b3 Rd8-c8 25. Nb3-d2 Rc8-b8 26. Ra1-a2 d5-d4 27. b2-b3 Rb8-b4 28. a4-a5 Re8-a8 29. a5-a6 Ra8-a7 30. Re1-e5 c5-c4 31. b3xc4 Be6xc4 32. Nd2xc4 Rb4xc4 33. Kg1-f1 Rc4-c6 34. Re5-a5 Rc6-e6 35. Ra5-a4 Kf8-e7 36. f2-f3 h6-h5 37. Kf1-f2 g7-g6 38. Ra4xd4 Ra7xa6 39. Ra2xa6 Re6xa6 40. Kf2-e3 Ra6-a5 41. Rd4-d3 Ke7-e6 42. Ke3-d2 Ra5-g5 43. f3-f4 Rg5-a5 44. Kd2-c3 Ke6-f5 45. Kc3-b4 Ra5-a1 46. c2-c4 Kf5-e4 47. Rd3-d7 Ra1-b1+ 48. Kb4-c5 Rb1-b3 49. Rd7-e7+ Ke4-f5 50. Re7xf7+ Kf5-e6 51. Rf7-a7 Rb3xg3 52. Ra7-a2 Rg3-c3 53. Kc5-d4 Rc3-c1 54. Ra2-a3 Rc1-f1 55. Kd4-e4 Rf1-e1+ 56. Ra3-e3 Re1-c1 57. g2-g3 h5-h4 58. g3xh4 Rc1-h1 59. Re3-g3 Ke6-f6 60. c4-c5 Rh1xh4 61. c5-c6 Rh4-h8 62. c6-c7 Rh8-c8 63. Rg3-c3 Kf6-e7 64. Ke4-f3 Ke7-f6 65. Kf3-g4 Kf6-f7 66. Kg4-g5 Kf7-g7 67. Rc3-c2 Kg7-h7 68. Rc2-c6 Kh7-h8 69. Kg5xg6 Kh8-g8 70. Rc6-d6 1-0",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Ng1-f3 b7-b6 4. g2-g3 Bc8-b7 5. Bf1-g2 Bf8-b4+ 6. Bc1-d2 Bb4xd2+ 7. Qd1xd2 O-O 8. Nb1-c3 d7-d6 9. O-O a7-a5 10. Qd2-c2 Nb8-d7 11. Rf1-d1 Rf8-e8 12. Ra1-c1 Qd8-b8 13. a2-a3 h7-h6 14. b2-b3 Nd7-f8 15. c4-c5 b6xc5 16. d4xc5 d6-d5 17. Nc3-a4 e6-e5 18. e2-e3 Bb7-c6 19. Rc1-b1 Qb8-b7 20. Nf3-e1 d5-d4 21. e3xd4 e5xd4 22. Bg2xc6 Qb7xc6 23. Rd1xd4 Nf8-e6 24. Rd4-d3 Ne6-g5 25. f2-f3 Nf6-g4 26. Qc2-c4 Ng4-e5 27. Qc4-d5 Qc6-g6 28. Rd3-d1 c7-c6 29. Qd5-d6 Ng5xf3+ 30. Ne1xf3 Ne5xf3+ 31. Kg1-g2 Re8-e2+ 0-1",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4-e5 Bc8-f5 4. Nb1-c3 Qd8-b6 5. g2-g4 Bf5-d7 6. f2-f4 h7-h5 7. f4-f5 e7-e6 8. g4xh5 e6xf5 9. Ng1-h3 Nb8-a6 10. Bf1xa6 Qb6xa6 11. Bc1-g5 Qa6-b6 12. Ra1-b1 f7-f6 13. e5xf6 g7xf6 14. Qd1-e2+ Bf8-e7 15. Bg5-e3 O-O-O 16. Nh3-f4 Be7-d6 17. O-O Ng8-h6 18. h2-h3 Rd8-e8 19. b2-b4 Rh8-g8+ 20. Kg1-h2 Qb6-c7 21. Qe2-d3 Rg8-g5 22. Qd3-e2 Rg5-g4 23. h3xg4 Re8xe3 0-1",
    "1. e2-e4 c7-c5 2. Ng1-f3 Ng8-f6 3. Nb1-c3 d7-d5 4. e4xd5 Nf6xd5 5. Bf1-b5+ Bc8-d7 6. Qd1-e2 Nd5-f6 7. d2-d4 c5xd4 8. Nf3xd4 g7-g6 9. Bc1-g5 Bf8-g7 10. O-O-O O-O 11. Rh1-e1 h7-h6 12. Bg5-h4 g6-g5 13. Bh4-g3 e7-e6 14. h2-h4 a7-a6 15. Bb5-d3 Qd8-b6 16. h4xg5 h6xg5 17. Bg3-e5 Bd7-c6 18. Be5xf6 Bg7xf6 19. Qe2-h5 Rf8-e8 20. Qh5-h7+ Kg8-f8 21. Nd4-f5 Nb8-d7 22. Qh7-h6+ Kf8-g8 23. Nf5-g3 Bf6-g7 24. Qh6-h7+ Kg8-f8 25. Ng3-h5 Bg7xc3 26. b2xc3 Qb6-c5 27. Bd3-g6 Qc5xf2 28. Re1-e2 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Nb8-c6 5. Nb1-c3 Qd8-c7 6. f2-f4 a7-a6 7. Nd4-f3 d7-d6 8. Bf1-d3 Ng8-f6 9. O-O Bf8-e7 10. Qd1-e1 O-O 11. a2-a3 Bc8-d7 12. b2-b4 Rf8-c8 13. Bc1-b2 Nf6-e8 14. Ra1-d1 b7-b5 15. Kg1-h1 Qc7-b6 16. e4-e5 Qb6-c7 17. Nc3-e4 d6-d5 18. Ne4-f6+ Ne8xf6 19. e5xf6 Be7xf6 20. Bb2xf6 g7xf6 21. Qe1-h4 Kg8-g7 22. Nf3-g5 f6xg5 23. Qh4xh7+ Kg7-f8 24. f4xg5 Kf8-e7 25. Qh7xf7+ Ke7-d6 26. g5-g6 Qc7-d8 27. Qf7-f4+ e6-e5 28. Qf4-g3 Bd7-e6 29. Bd3-f5 Qd8-e7 30. Bf5xe6 Qe7xe6 31. Qg3-g5 Rc8-f8 32. h2-h4 Qe6-e7 33. Qg5-g4 Qe7-e6 34. Qg4-g5 Qe6-e7 35. Qg5-g4 Qe7-e6 36. Qg4-g5 Qe6-e7 1/2-1/2",
    "1. Ng1-f3 Ng8-f6 2. c2-c4 b7-b6 3. g2-g3 Bc8-b7 4. Bf1-g2 c7-c5 5. O-O e7-e6 6. Nb1-c3 Bf8-e7 7. b2-b3 O-O 8. Bc1-b2 d7-d5 9. Nf3-e5 Nb8-d7 10. d2-d4 c5xd4 11. Qd1xd4 Be7-c5 12. Qd4-f4 Bc5-d6 13. Ne5xf7 Bd6xf4 14. Nf7xd8 Ra8xd8 15. g3xf4 Nd7-c5 16. b3-b4 d5-d4 17. b4xc5 Bb7xg2 18. Kg1xg2 d4xc3 19. Bb2xc3 Nf6-e4 20. Bc3-e5 Ne4xc5 21. Rf1-d1 Rd8-d7 22. Rd1-d6 Rd7xd6 23. Be5xd6 Rf8-c8 24. Ra1-d1 Kg8-f7 25. Bd6-e5 g7-g6 26. Kg2-f3 Kf7-e7 27. Kf3-g4 h7-h6 28. Be5-g7 Nc5-e4 29. Kg4-f3 Rc8xc4 30. Bg7xh6 b6-b5 31. h2-h4 b5-b4 32. Bh6-g5+ Ne4xg5+ 33. h4xg5 Rc4-c2 34. Rd1-a1 a7-a5 35. Kf3-e3 Ke7-d6 36. Ra1-d1+ Kd6-c7 37. Rd1-d2 Rc2-c4 38. Rd2-d3 Rc4-c2 39. a2-a3 Kc7-c6 40. a3xb4 a5xb4 41. Rd3-d8 Kc6-b5 42. Ke3-d3 b4-b3 43. Rd8-b8+ Kb5-a4 44. Rb8-b6 Rc2-c8 45. e2-e4 Rc8-f8 46. Kd3-c4 Rf8-a8 47. Rb6-b4+ 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 d7-d6 5. O-O Bc8-d7 6. c2-c4 g7-g6 7. d2-d4 Bf8-g7 8. Bc1-e3 e5xd4 9. Nf3xd4 Ng8-f6 10. Nb1-c3 O-O 11. Nd4xc6 Bd7xc6 12. Ba4xc6 b7xc6 13. f2-f3 Nf6-d7 14. Qd1-a4 Bg7xc3 15. b2xc3 c6-c5 16. Qa4-c2 Qd8-e7 17. Ra1-e1 Rf8-e8 18. Be3-h6 f7-f6 19. f3-f4 Qe7-f7 20. Qc2-d3 Re8-e7 21. f4-f5 g6-g5 22. h2-h4 g5-g4 23. Qd3-d5 Qf7xd5 24. c4xd5 Ra8-e8 25. Rf1-f4 Nd7-e5 26. Re1-e2 Kg8-h8 27. Rf4-f2 Ne5-f7 28. Bh6-f4 Re7xe4 29. Re2-b2 Re4-c4 30. Rb2-b7 Re8-e7 31. h4-h5 h7-h6 32. Rb7-b3 a6-a5 33. Kg1-h2 a5-a4 34. Rb3-b8+ Kh8-g7 35. Bf4-d2 Nf7-g5 36. Rf2-f4 g4-g3+ 37. Kh2xg3 Ng5-e4+ 0-1",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-c3 Bf8-b4 4. e4-e5 Qd8-d7 5. a2-a3 Bb4xc3+ 6. b2xc3 b7-b6 7. Qd1-g4 f7-f5 8. Qg4-g3 Bc8-a6 9. Bf1xa6 Nb8xa6 10. Ng1-e2 O-O-O 11. Ne2-f4 Na6-b8 12. a3-a4 Nb8-c6 13. a4-a5 Nc6xa5 14. Ra1xa5 b6xa5 15. O-O Kc8-b8 16. Bc1-a3 Ng8-e7 17. Ba3xe7 Qd7xe7 18. Qg3-d3 c7-c6 19. Qd3-a6 Kb8-a8 20. Qa6xc6+ Qe7-b7 21. Qc6xe6 Rd8-c8 22. g2-g3 Rc8xc3 23. Nf4xd5 Rc3xc2 24. Nd5-e3 Rc2-b2 25. Qe6xf5 a5-a4 26. e5-e6 a4-a3 27. Qf5-a5 a3-a2 28. d4-d5 Rh8-e8 29. Kg1-g2 Rb2-b5 30. Qa5-a3 Rb5-b3 31. Qa3-a5 Rb3-b5 32. Qa5-a3 Rb5-b6 33. Kg2-g1 Rb6-a6 34. Qa3-c3 Re8-c8 35. Qc3-e5 Qb7-b5 36. Rf1-a1 Ra6-a3 37. e6-e7 Ra3-a6 38. Qe5-f5 Qb5-e8 39. Qf5-e4 Ra6-a4 40. Qe4-e6 Rc8-b8 41. d5-d6 Qe8-c6 42. Qe6-d5 Qc6xd5 43. Ne3xd5 1-0",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Ng1-f3 b7-b6 4. g2-g3 Bf8-b4+ 5. Bc1-d2 Bb4xd2+ 6. Qd1xd2 Bc8-a6 7. b2-b3 c7-c6 8. Qd2-b2 d7-d5 9. Nb1-d2 O-O 10. Bf1-g2 Qd8-e7 11. O-O Nb8-d7 12. Rf1-e1 Ra8-c8 13. e2-e4 d5xe4 14. Nd2xe4 c6-c5 15. Ne4xf6+ Qe7xf6 16. Nf3-e5 Rc8-c7 17. Qb2-a3 Ba6-c8 18. Ne5-c6 Bc8-b7 19. d4-d5 Bb7xc6 20. d5xc6 Nd7-e5 21. Ra1-d1 Ne5-f3+ 22. Bg2xf3 Qf6xf3 23. Qa3-a4 Qf3xc6 24. Qa4xc6 Rc7xc6 25. Rd1-d7 a7-a6 26. Re1-d1 g7-g5 27. Kg1-g2 Kg8-g7 28. Kg2-f3 Kg7-f6 29. Kf3-e4 b6-b5 30. Rd1-c1 Rf8-b8 31. Rc1-c3 Kf6-g6 32. h2-h4 h7-h6 33. h4-h5+ Kg6xh5 34. Rd7xf7 b5xc4 35. Rf7-f6 c4xb3 36. g3-g4+ Kh5xg4 37. Rc3-g3+ Kg4-h5 38. Rg3-h3+ Kh5-g4 39. Rf6xh6 Rb8-b4+ 40. Ke4-e5 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Nf6xe4 6. d2-d4 b7-b5 7. Ba4-b3 d7-d5 8. d4xe5 Bc8-e6 9. Nb1-d2 Ne4-c5 10. c2-c3 d5-d4 11. Bb3xe6 Nc5xe6 12. c3xd4 Nc6xd4 13. a2-a4 Bf8-e7 14. Nf3xd4 Ne6xd4 15. Nd2-e4 O-O 16. a4xb5 Nd4xb5 17. Bc1-e3 Qd8-c8 18. Qd1-c2 Qc8-e6 19. f2-f4 Ra8-d8 20. Ra1-a4 Rd8-d7 21. Rf1-a1 Qe6-d5 22. h2-h3 f7-f6 23. e5xf6 Be7xf6 24. Ne4xf6+ Rf8xf6 25. Ra4xa6 Rf6xa6 26. Ra1xa6 Nb5-d4 27. Qc2-a4 Rd7-d8 28. Be3xd4 Qd5xd4+ 29. Qa4xd4 Rd8xd4 30. Ra6-c6 Rd4-d2 31. b2-b3 Rd2-d3 32. b3-b4 Rd3-d4 33. b4-b5 Rd4-d5 34. Rc6xc7 Rd5xb5 35. Kg1-f2 g7-g5 36. f4xg5 1/2-1/2",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 c7-c5 4. Ng1-f3 Ng8-f6 5. e4xd5 Nf6xd5 6. Nd2-b3 Nb8-d7 7. Bf1-e2 Bf8-e7 8. O-O O-O 9. c2-c4 Nd5-f6 10. Bc1-f4 b7-b6 11. Qd1-d2 1/2-1/2",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 c7-c5 4. e4xd5 e6xd5 5. Ng1-f3 Ng8-f6 6. d4xc5 Bf8xc5 7. Nd2-b3 Bc5-e7 8. Bf1-e2 O-O 9. O-O Bc8-f5 10. Rf1-e1 Nb8-c6 11. c2-c3 Nf6-e4 12. Bc1-e3 Bf5-g6 13. Nb3-d4 Be7-f6 14. Qd1-a4 Qd8-c7 15. Ra1-c1 Rf8-d8 16. Be2-f1 a7-a6 17. g2-g3 b7-b5 18. Qa4-d1 Nc6-a5 19. a2-a4 b5xa4 20. Qd1xa4 Na5-c4 21. Be3-f4 Qc7-c8 22. b2-b3 Nc4-d6 23. Qa4-a3 Nd6-b5 24. Qa3-b2 Qc8-b7 25. Nd4xb5 a6xb5 26. Bf4-e5 b5-b4 27. Be5xf6 Ne4xf6 28. c3xb4 Qb7xb4 29. Nf3-d4 Rd8-c8 30. Re1-d1 h7-h6 31. Bf1-h3 Rc8xc1 32. Rd1xc1 Bg6-d3 33. Nd4-c6 Qb4-b6 34. Nc6-d4 Ra8-a4 35. Nd4-c6 Bd3-e4 36. b3-b4 d5-d4 37. b4-b5 d4-d3 38. Bh3-f1 Nf6-g4 39. h2-h3 Ng4-f6 40. h3-h4 Ra4-a8 41. Nc6-e5 Ra8-d8 42. Rc1-d1 1/2-1/2",
    "1. e2-e4 d7-d6 2. d2-d4 g7-g6 3. Nb1-c3 Bf8-g7 4. Bc1-e3 a7-a6 5. Qd1-d2 b7-b5 6. h2-h4 h7-h5 7. f2-f3 Ng8-f6 8. O-O-O Nb8-d7 9. Ng1-h3 Nd7-b6 10. Bf1-d3 b5-b4 11. Nc3-e2 a6-a5 12. Nh3-f2 c7-c6 13. Kc1-b1 Qd8-c7 14. e4-e5 Nf6-d5 15. e5xd6 e7xd6 16. Be3-g5 Bc8-e6 17. Nf2-e4 Ke8-d7 18. Bg5-f4 Nd5xf4 19. Ne2xf4 Nb6-d5 20. Nf4xe6 f7xe6 21. Ne4-g5 Bg7xd4 22. Bd3xg6 Bd4-e3 23. Qd2-d3 Be3xg5 24. h4xg5 Nd5-f4 25. Qd3-e4 d6-d5 26. g2-g3 Nf4xg6 27. Qe4xg6 Qc7xg3 28. Qg6-f7+ Kd7-d6 29. Rd1-e1 Ra8-e8 30. g5-g6 Rh8-f8 31. Re1xe6+ Re8xe6 32. Qf7xf8+ Kd6-d7 33. Qf8-f7+ Kd7-d6 34. Qf7-f8+ 1/2-1/2",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Ng8-f6 5. Ne4xf6+ g7xf6 6. c2-c3 Bc8-f5 7. Ng1-f3 e7-e6 8. g2-g3 Nb8-d7 9. Bf1-g2 Bf8-g7 10. O-O O-O 11. Nf3-h4 Bf5-g6 12. a2-a4 a7-a5 13. Bc1-f4 Nd7-b6 14. Qd1-b3 Bg6-d3 15. Rf1-d1 Bd3-c4 16. Qb3-c2 Nb6-d5 17. b2-b3 Nd5xf4 18. b3xc4 Nf4xg2 19. Nh4xg2 Qd8-c7 20. Ra1-b1 b7-b6 21. d4-d5 c6xd5 22. c4xd5 Ra8-c8 23. d5xe6 f7xe6 24. Ng2-f4 Rf8-e8 25. Qc2-b3 Bg7-h6 26. Nf4xe6 Qc7xc3 27. Qb3xc3 Rc8xc3 28. Rb1xb6 Rc3-c4 29. Rd1-d7 Rc4-c1+ 30. Kg1-g2 Rc1-c2 31. Kg2-f3 Rc2-a2 32. Ne6-c7 Re8-e2 33. Rb6-b8+ Bh6-f8 34. Kf3-g4 Re2xf2 35. h2-h4 Ra2xa4+ 36. Kg4-h3 Ra4-a2 37. Nc7-e6 1-0",
    "1. e2-e4 d7-d5 2. e4xd5 Ng8-f6 3. c2-c4 c7-c6 4. d5xc6 Nb8xc6 5. Ng1-f3 e7-e5 6. d2-d3 e5-e4 7. d3xe4 Qd8xd1+ 8. Ke1xd1 Nf6xe4 9. Bc1-e3 Bc8-f5 10. Nf3-h4 O-O-O++ 11. Kd1-c1 Bf5-e6 12. Nb1-c3 Ne4xc3 13. b2xc3 b7-b6 14. Nh4-f3 Bf8-c5 15. Be3xc5 b6xc5 16. Ra1-b1 Rh8-e8 17. Bf1-e2 Be6-h3 18. g2xh3 Re8xe2 19. Rb1-b2 Re2xb2 20. Kc1xb2 Rd8-d3 21. Nf3-g5 Rd3-d2+ 22. Kb2-a3 Rd2xf2 23. Rh1-d1 h7-h6 24. Ng5-e4 Rf2xh2 25. Rd1-g1 Rh2-e2 26. Ne4xc5 g7-g6 27. Rg1-g3 Nc6-e5 28. Ka3-b3 Kc8-c7 29. Nc5-a6+ Kc7-b6 30. Na6-b4 Re2-e4 31. Rg3-g1 a7-a5 32. Nb4-d5+ Kb6-c5 33. Rg1-d1 Ne5xc4 34. Nd5-f6 Re4-f4 35. Nf6-d7+ Kc5-c6 36. a2-a4 g6-g5 37. Nd7-b8+ Kc6-c7 38. Nb8-d7 Nc4-d6 39. Nd7-e5 Rf4-f5 40. Ne5-g4 h6-h5 41. Ng4-e3 Rf5-f3 42. Ne3-d5+ Kc7-c6 43. h3-h4 g5xh4 44. Nd5-e7+ Kc6-d7 45. Ne7-c8 Kd7xc8 46. Rd1xd6 h4-h3 47. Rd6-d4 h3-h2 48. Rd4-h4 Rf3-f2 49. Rh4xh5 f7-f5 50. Kb3-c4 Rf2-d2 51. Kc4-c5 f5-f4 52. Kc5-c6 Kc8-d8 53. c3-c4 f4-f3 54. c4-c5 Rd2-g2 55. Kc6-b6 Rg2-b2+ 56. Kb6-c6 f3-f2 0-1",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4xd5 c6xd5 4. c2-c4 Ng8-f6 5. Nb1-c3 g7-g6 6. Qd1-b3 Bf8-g7 7. c4xd5 O-O 8. Bf1-e2 Nb8-d7 9. Be2-f3 Nd7-b6 10. Ng1-e2 Bc8-f5 11. O-O a7-a5 12. Bc1-f4 Bf5-d3 13. d5-d6 e7xd6 14. Qb3-d1 Bd3-a6 15. b2-b3 h7-h6 16. h2-h3 Nf6-h7 17. g2-g3 Ra8-c8 18. Bf3-g2 Rf8-e8 19. Rf1-e1 Nh7-f6 20. Ra1-c1 Qd8-d7 21. Qd1-d2 Kg8-h7 22. Bf4-e3 d6-d5 23. Ne2-f4 Rc8-c7 24. Nf4-d3 Ba6xd3 25. Qd2xd3 Rc7-c8 26. a2-a4 Bg7-f8 27. Be3-d2 Re8xe1+ 28. Rc1xe1 Qd7-d8 29. Nc3-e2 Nb6-a8 30. Ne2-f4 b7-b6 31. Qd3-b5 Rc8-c2 32. Bd2-e3 Bf8-b4 33. Re1-c1 Rc2xc1+ 34. Be3xc1 Kh7-g7 35. Bc1-e3 Bb4-d6 36. Nf4xd5 Na8-c7 37. Nd5xc7 Qd8xc7 38. Qb5-c6 Qc7-e7 39. Qc6xb6 Bd6xg3 40. Qb6xa5 Bg3-d6 41. Qa5-b5 g6-g5 42. a4-a5 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 Nb8-c6 6. Nd4xc6 b7xc6 7. e4-e5 Nf6-d5 8. Nc3-e4 Qd8-c7 9. f2-f4 Qc7-b6 10. Bf1-d3 Bc8-a6 11. Bd3xa6 Qb6xa6 12. Qd1-e2 Qa6xe2+ 13. Ke1xe2 f7-f5 14. Ne4-g5 Ra8-b8 15. a2-a3 h7-h6 16. Ng5-f3 g7-g5 17. f4xg5 h6xg5 18. Nf3xg5 Rh8-h4 19. g2-g3 Rh4-g4 20. Ng5-f3 Rg4-e4+ 21. Ke2-d3 c6-c5 22. b2-b3 d7-d6 23. Nf3-g5 Re4-d4+ 24. Kd3-e2 Ke8-d7 25. e5xd6 c5-c4 26. b3-b4 Rd4-g4 27. Ke2-f2 Bf8xd6 28. Ng5-f7 Bd6-c7 29. Rh1-e1 Rb8-f8 30. Nf7-e5+ Bc7xe5 31. Re1xe5 Rf8-h8 32. Kf2-g2 Kd7-d6 33. Re5-e2 Rg4-d4 34. Bc1-g5 e6-e5 35. Ra1-e1 e5-e4 36. h2-h4 Kd6-e5 37. Re2-f2 c4-c3 38. Re1-f1 Rh8-f8 39. h4-h5 e4-e3 40. Rf2-e2 f5-f4 41. Bg5xf4+ Nd5xf4+ 42. g3xf4+ Ke5-e4 43. Rf1-e1 Rd4-d2 44. Kg2-g1 Rf8-g8+ 45. Kg1-f1 Rg8-g3 46. h5-h6 Rg3-h3 0-1",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 Bf8-c5 4. c2-c3 Ng8-f6 5. O-O O-O 6. d2-d4 Bc5-b6 7. Bc1-g5 h7-h6 8. Bg5-h4 d7-d6 9. Bb5xc6 b7xc6 10. d4xe5 d6xe5 11. Nb1-d2 Bc8-g4 12. Qd1-e2 g7-g5 13. Bh4-g3 Nf6-d7 14. h2-h3 Bg4-h5 15. Qe2-c4 Kg8-h7 16. Qc4xc6 Nd7-c5 17. Bg3xe5 f7-f5 18. Nf3-d4 Qd8-e8 19. Nd2-c4 f5xe4 20. Qc6xe8 Rf8xe8 21. Be5-g3 Nc5-d3 22. Nc4xb6 c7xb6 23. b2-b3 a7-a6 24. a2-a4 Re8-d8 25. Nd4-c2 Ra8-c8 26. c3-c4 Nd3-c5 27. Rf1-b1 Rd8-d3 28. a4-a5 b6xa5 29. Ra1xa5 Rd3xb3 30. Rb1xb3 Nc5xb3 31. Ra5xa6 Rc8xc4 32. Nc2-e3 Rc4-c1+ 33. Kg1-h2 Nb3-d2 34. Bg3-e5 Nd2-f1+ 35. Ne3xf1 Rc1xf1 36. g2-g4 Bh5-f7 37. Kh2-g2 Rf1-d1 38. Ra6-a7 1/2-1/2",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Nb1-c3 Bf8-b4 4. Qd1-c2 O-O 5. a2-a3 Bb4xc3+ 6. Qc2xc3 Qd8-e8 7. f2-f3 d7-d6 8. e2-e4 e6-e5 9. Bc1-e3 Nb8-c6 10. Ng1-e2 a7-a5 11. d4-d5 Nc6-e7 12. h2-h4 a5-a4 13. h4-h5 Nf6-d7 14. g2-g4 b7-b6 15. Ne2-g3 Nd7-c5 16. Ra1-c1 f7-f6 17. Rc1-c2 Nc5-b3 18. Ng3-f5 Ra8-a7 19. Rc2-g2 Qe8-d8 20. Nf5xe7+ 1/2-1/2",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. g2-g3 Bf8-b4+ 4. Bc1-d2 Qd8-e7 5. Ng1-f3 Nb8-c6 6. Nb1-c3 d7-d5 7. a2-a3 Bb4xc3 8. Bd2xc3 Nf6-e4 9. Qd1-c2 O-O 10. Bf1-g2 Rf8-d8 11. O-O a7-a5 12. b2-b3 Ne4xc3 13. Qc2xc3 e6-e5 14. d4xe5 d5-d4 15. Qc3-b2 Bc8-g4 16. Rf1-d1 Qe7-c5 17. Rd1-d2 a5-a4 18. b3-b4 Qc5xc4 19. Ra1-c1 Qc4-b3 20. Qb2-a1 Bg4-f5 21. Nf3-h4 Bf5-d7 22. Rd2-d3 Qb3-e6 23. Nh4-f3 Nc6xe5 24. Nf3xe5 Qe6xe5 25. Qa1xd4 Qe5xd4 26. Rd3xd4 Bd7-b5 27. Bg2xb7 Rd8xd4 28. Bb7xa8 Bb5xe2 29. Rc1xc7 g7-g5 30. f2-f3 Kg8-g7 31. Kg1-f2 Be2-a6 32. Rc7-c6 Ba6-d3 33. Rc6-c5 Bd3-g6 34. Rc5-d5 Rd4-c4 35. Rd5-c5 Rc4-d4 36. Rc5-d5 Rd4-c4 37. Rd5xg5 Rc4-c3 38. Rg5-a5 Rc3xa3 39. Kf2-e1 Ra3-e3+ 40. Ke1-d2 Re3-d3+ 41. Kd2-e2 Rd3-b3 42. Ra5xa4 Rb3-b2+ 43. Ke2-d1 Rb2xh2 44. Ra4-a6 Bg6-d3 45. Ra6-d6 Bd3-b5 46. Kd1-c1 h7-h5 47. Rd6-d4 Kg7-h6 48. Rd4-h4 Rh2-f2 49. g3-g4 Bb5-a4 50. Rh4xh5+ Kh6-g7 51. Ba8-e4 Kg7-f6 52. b4-b5 1-0",
    "1. Ng1-f3 Ng8-f6 2. c2-c4 b7-b6 3. Nb1-c3 c7-c5 4. e2-e4 d7-d6 5. d2-d4 c5xd4 6. Nf3xd4 Bc8-b7 7. Nc3-d5 e7-e6 8. Nd5xf6+ Qd8xf6 9. Nd4-b5 Qf6-d8 10. Bf1-e2 a7-a6 11. Nb5-c3 Bf8-e7 12. Bc1-f4 O-O 13. O-O Qd8-c7 14. Qd1-d2 Nb8-d7 15. Rf1-d1 Nd7-e5 16. Ra1-c1 Rf8-d8 17. Bf4-e3 Ne5-d7 18. b2-b3 Ra8-c8 19. Be3-g5 Be7xg5 20. Qd2xg5 Nd7-f6 21. Be2-f3 h7-h6 22. Qg5-e3 Qc7-c5 23. Qe3-d2 Qc5-e5 24. Qd2-e3 Qe5-c5 25. Qe3-d2 1/2-1/2",
    "1. d2-d4 d7-d6 2. e2-e4 Ng8-f6 3. Bf1-d3 e7-e5 4. c2-c3 d6-d5 5. d4xe5 Nf6xe4 6. Ng1-f3 Nb8-c6 7. Nb1-d2 Ne4-c5 8. Nd2-b3 Bc8-g4 9. O-O Qd8-d7 10. Rf1-e1 O-O-O 11. Nb3xc5 Bf8xc5 12. b2-b4 Bc5-b6 13. a2-a4 a7-a5 14. b4-b5 Nc6-e7 15. Bc1-e3 Bb6xe3 16. Re1xe3 Kc8-b8 17. Qd1-e1 Bg4xf3 18. Re3xf3 Ne7-g6 19. Ra1-d1 Qd7-e6 20. Rf3-e3 Rh8-e8 21. Bd3xg6 h7xg6 22. h2-h3 g6-g5 23. Rd1-d4 Qe6-f5 24. Kg1-h2 1/2-1/2",
    "1. e2-e4 c7-c5 2. Ng1-f3 Nb8-c6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 e7-e6 6. Nd4xc6 b7xc6 7. e4-e5 Nf6-d5 8. Nc3-e4 Qd8-c7 9. f2-f4 Qc7-b6 10. c2-c4 Nd5-e3 11. Qd1-d3 Bf8-b4+ 12. Bc1-d2 a7-a5 13. b2-b3 O-O 14. Bd2xb4 Qb6xb4+ 15. Ke1-e2 Ne3xc4 16. Ke2-f3 Nc4-b6 17. a2-a3 Qb4-e7 18. Qd3-d6 Qe7xd6 19. e5xd6 Bc8-a6 20. Ne4-c5 Ba6xf1 21. Rh1xf1 Ra8-a7 22. Ra1-c1 f7-f6 23. g2-g4 Kg8-f7 24. h2-h4 Rf8-e8 25. Rf1-e1 Ra7-a8 26. h4-h5 h7-h6 27. Rc1-c2 Re8-h8 28. Rc2-h2 Ra8-a7 29. a3-a4 Nb6-d5 30. Re1-g1 Nd5-b4 31. g4-g5 h6xg5 32. f4xg5 Nb4-a6 33. Nc5-e4 Ra7-b7 34. g5-g6+ Kf7-g8 35. h5-h6 Rb7xb3+ 36. Kf3-f4 e6-e5+ 37. Kf4-g4 g7xh6 38. Kg4-h5 Kg8-g7 39. Rh2-f2 Rh8-f8 40. Ne4-g3 Rb3xg3 41. Rg1xg3 Na6-c5 42. Rf2-b2 f6-f5 43. Rg3-c3 Nc5-e4 44. Rb2-b7 Ne4-f6+ 45. Kh5-h4 Kg7xg6 46. Rc3-c5 e5-e4 47. Rc5xa5 f5-f4 48. Ra5-e5 e4-e3 49. Rb7-b1 Nf6-d5 50. Rb1-f1 f4-f3 51. Kh4-g3 f3-f2 52. Re5-e4 h6-h5 53. Re4-e5 Rf8-a8 54. Re5-e4 Kg6-f5 55. Kg3-f3 Ra8-g8 56. Re4-h4 Kf5-e6 57. Rh4xh5 Ke6xd6 58. Rh5xd5+ Kd6xd5 59. Kf3xe3 Rg8-a8 60. Rf1-a1 Ra8xa4 0-1",
    "1. d2-d4 Ng8-f6 2. Ng1-f3 e7-e6 3. c2-c4 b7-b6 4. a2-a3 Bc8-a6 5. Qd1-c2 Ba6-b7 6. Bc1-g5 h7-h6 7. Bg5xf6 Qd8xf6 8. e2-e4 c7-c5 9. d4-d5 d7-d6 10. Nb1-c3 e6-e5 11. Bf1-e2 g7-g6 12. b2-b4 Nb8-d7 13. Qc2-a4 Qf6-d8 14. Nf3-d2 h6-h5 15. Ra1-b1 Bf8-h6 16. b4xc5 Bh6xd2+ 17. Ke1xd2 d6xc5 18. Nc3-b5 O-O 19. Nb5-d6 Qd8-c7 20. Nd6xb7 Qc7xb7 21. g2-g4 h5xg4 22. Be2xg4 Nd7-f6 23. f2-f3 Kg8-g7 24. h2-h4 Rf8-h8 25. h4-h5 g6xh5 26. Bg4-f5 Kg7-f8 27. Rb1-g1 Kf8-e7 28. f3-f4 Qb7-c7 29. Qa4-c6 Qc7xc6 30. d5xc6 e5xf4 31. e4-e5 Ra8-d8+ 32. Kd2-c3 Nf6-e8 33. Rg1-g5 Rd8-d4 34. Rh1xh5 Rh8xh5 35. Rg5xh5 f4-f3 36. Bf5-d7 Ne8-g7 37. Rh5-h7 f3-f2 38. Bd7-h3 Ng7-e6 39. Bh3xe6 Rd4-f4 40. Rh7-h1 f7xe6 0-1",
    "1. c2-c4 Ng8-f6 2. g2-g3 e7-e5 3. Bf1-g2 c7-c6 4. d2-d4 e5xd4 5. Qd1xd4 d7-d5 6. Ng1-f3 Bf8-e7 7. c4xd5 c6xd5 8. O-O Nb8-c6 9. Qd4-a4 O-O 10. Bc1-e3 Nf6-e4 11. Nb1-c3 Ne4xc3 12. b2xc3 Be7-f6 13. Be3-d4 Rf8-e8 14. Bd4xf6 Qd8xf6 15. Nf3-d4 Bc8-d7 16. e2-e3 Nc6xd4 17. Qa4xd4 Bd7-e6 18. Rf1-d1 Re8-c8 19. Qd4xf6 g7xf6 20. Bg2xd5 Be6xd5 21. Rd1xd5 Rc8xc3 22. Rd5-d7 b7-b5 23. Rd7-b7 a7-a6 24. a2-a4 Ra8-c8 25. a4xb5 Rc3-c1+ 26. Ra1xc1 Rc8xc1+ 27. Kg1-g2 a6xb5 28. Rb7xb5 Kg8-g7 29. Kg2-f3 Rc1-c4 30. Rb5-d5 Rc4-c2 31. Rd5-d4 f6-f5 32. h2-h3 Kg7-g6 33. Rd4-d5 h7-h5 34. Rd5-d6+ Kg6-g7 35. Kf3-g2 Rc2-b2 36. Kg2-f1 Rb2-a2 37. Rd6-d4 Kg7-g6 38. Kf1-e1 Ra2-b2 39. Rd4-d6+ Kg6-g7 40. Rd6-d2 Rb2-b3 41. Ke1-e2 Kg7-g6 42. Ke2-f3 Rb3-b4 43. h3-h4 Rb4-a4 44. Kf3-e2 Ra4-a3 45. Rd2-c2 Kg6-f6 46. Ke2-d2 Kf6-g6 47. Kd2-c1 Kg6-g7 48. Rc2-d2 Ra3-c3+ 49. Kc1-b2 Rc3-c6 50. Rd2-d5 Kg7-f6 51. Rd5-d4 Kf6-g6 52. Kb2-b3 Kg6-f6 53. Rd4-c4 Rc6-a6 54. Rc4-f4 Kf6-g6 55. Kb3-c4 Ra6-a4+ 56. Kc4-b3 Ra4-a6 57. Rf4-a4 Ra6-c6 58. Ra4-b4 Kg6-f6 59. Rb4-b8 Kf6-g6 60. Rb8-b5 Kg6-f6 61. Rb5-d5 Rc6-c7 62. Rd5-d6+ Kf6-g7 63. Rd6-d4 Rc7-c6 64. Rd4-c4 Rc6-a6 65. Kb3-b2 Kg7-g6 66. Rc4-c5 Ra6-a7 67. Kb2-c1 Ra7-a2 68. Rc5-c6+ Kg6-g7 69. Rc6-c2 Ra2-a3 70. Kc1-d2 Kg7-g6 71. Kd2-e2 Kg6-f6 72. Rc2-c6+ Kf6-g7 73. e3-e4 f5xe4 74. Rc6-c5 Kg7-g6 75. Rc5-g5+ Kg6-h6 76. Rg5-e5 Kh6-g6 77. Re5xe4 f7-f6 78. Re4-d4 Ra3-b3 79. Rd4-d3 Rb3-b4 80. Ke2-e3 Rb4-a4 81. Rd3-d4 Ra4-a2 82. f2-f3 Ra2-a3+ 83. Ke3-f4 Ra3-a1 84. Rd4-b4 Ra1-c1 85. g3-g4 h5xg4 86. f3xg4 Rc1-f1+ 87. Kf4-g3 Rf1-g1+ 88. Kg3-f3 Rg1-f1+ 89. Kf3-g2 Rf1-c1 90. Rb4-f4 Rc1-c3 91. Rf4-f5 Rc3-b3 92. h4-h5+ Kg6-g7 93. Rf5-f3 Rb3-b4 94. Kg2-g3 Rb4-c4 95. Rf3-b3 Rc4-c7 96. Rb3-b4 Rc7-c3+ 97. Kg3-f4 Rc3-c7 98. Rb4-b5 Rc7-c4+ 99. Kf4-f5 Rc4-c7 100. Kf5-e6 Rc7-c6+ 101. Ke6-d5 Rc6-c7 102. Kd5-e6 Rc7-c6+ 103. Ke6-f5 Rc6-c7 104. Rb5-d5 Kg7-f7 105. Rd5-d8 Rc7-c5+ 106. Kf5-e4 Rc5-c4+ 1/2-1/2",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4-e5 Bc8-f5 4. Nb1-c3 Qd8-b6 5. g2-g4 Bf5-d7 6. Nc3-a4 Qb6-c7 7. Na4-c5 e7-e6 8. Nc5-d3 c6-c5 9. d4xc5 Bd7-b5 10. f2-f4 Bf8xc5 11. Ng1-f3 Ng8-e7 12. a2-a4 Bb5xd3 13. Bf1xd3 Nb8-c6 14. Qd1-e2 h7-h5 15. g4-g5 g7-g6 16. c2-c3 a7-a6 17. Bc1-e3 Bc5xe3 18. Qe2xe3 O-O 19. O-O Ra8-c8 20. Ra1-d1 Rf8-d8 21. Qe3-f2 Nc6-a5 22. Nf3-d4 Na5-c4 23. Rd1-c1 Qc7-b6 24. b2-b3 Nc4-a5 25. b3-b4 Na5-c6 26. Nd4-b3 d5-d4 27. b4-b5 a6xb5 28. a4xb5 Nc6-a5 29. Nb3-d2 Kg8-g7 30. c3-c4 Ne7-f5 31. Rc1-a1 Nf5-e3 32. Rf1-c1 Rc8-c7 33. Kg1-h1 Rd8-c8 34. Ra1-a4 Qb6xb5 35. c4xb5 Rc7xc1+ 36. Nd2-f1 Na5-b3 37. Ra4xd4 Ne3xf1 38. Kh1-g2 Nb3xd4 39. Qf2xd4 Rc1-d1 40. Qd4-e4 Rd1-d2+ 41. Bd3-e2 Nf1xh2 42. Kg2xh2 Rc8-c2 43. Kh2-g3 Rd2xe2 44. Qe4-d3 Re2-g2+ 45. Kg3-f3 h5-h4 46. Qd3-f1 Rg2-g3+ 47. Kf3-e4 h4-h3 48. f4-f5 g6xf5+ 0-1",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Nb8-d7 5. Bf1-c4 Ng8-f6 6. Ne4-g5 e7-e6 7. Qd1-e2 Nd7-b6 8. Bc4-b3 h7-h6 9. Ng5-f3 a7-a5 10. a2-a4 c6-c5 11. Bc1-f4 Bf8-d6 12. Nf3-e5 c5xd4 13. O-O-O Nb6-d5 14. Bf4-g3 O-O 15. Rd1xd4 b7-b6 16. Bb3-c4 Bc8-b7 17. Ng1-f3 Qd8-c7 18. Bc4-b5 Nf6-h5 19. Kc1-b1 Ra8-c8 20. Rd4-c4 Nh5xg3 21. h2xg3 Qc7-e7 22. Rc4-h4 Rf8-d8 23. Ne5-g4 Nd5-f6 24. Ng4xh6+ g7xh6 25. Rh4xh6 Bb7-e4 26. Bb5-d3 Be4xd3 27. c2xd3 Kg8-f8 28. Rh6-h8+ Nf6-g8 29. g3-g4 Bd6-f4 30. g2-g3 Bf4-g5 31. Rh8-h7 Rd8-d5 32. Nf3-e5 Rd5xe5 33. Qe2xe5 Bg5-f6 34. Qe5-e2 Qe7-b4 35. g4-g5 Bf6xg5 36. Qe2-h5 Qb4-e7 37. Qh5-g4 Qe7-c5 38. Qg4-e2 Bg5-f6 39. f2-f4 Ng8-e7 40. g3-g4 Ne7-d5 41. g4-g5 Bf6-g7 42. Qe2-h2 Nd5-c3+ 43. b2xc3 Qc5xc3 44. Kb1-a2 Rc8-c5 45. Rh7xg7 1/2-1/2",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 f7-f5 4. d2-d3 f5xe4 5. d3xe4 Ng8-f6 6. O-O 1/2-1/2",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 Ng8-f6 4. e4-e5 Nf6-d7 5. c2-c3 c7-c5 6. f2-f4 Nb8-c6 7. Nd2-f3 c5xd4 8. c3xd4 Qd8-b6 9. g2-g3 Bf8-b4+ 10. Ke1-f2 g7-g5 11. f4xg5 Nd7xe5 12. Nf3xe5 Nc6xe5 13. Bc1-e3 Ne5-c6 14. Ng1-f3 Bb4-f8 15. Qd1-d2 Bf8-g7 16. Kf2-g2 Bc8-d7 17. Ra1-c1 Qb6-a5 18. Rc1-c3 e6-e5 19. d4xe5 d5-d4 20. Be3xd4 Nc6xd4 21. Qd2xd4 Ra8-d8 22. Qd4-f4 Qa5xa2 23. Qf4-c1 Qa2-e6 24. h2-h3 O-O 25. Bf1-c4 Qe6-e7 26. Rh1-e1 Bd7-e6 27. Qc1-c2 Qe7-d7 28. Bc4xe6 f7xe6 29. Re1-e2 Qd7-f7 30. Qc2-e4 Rd8-d5 31. Rc3-c1 Rd5-b5 32. Qe4-e3 a7-a5 33. h3-h4 a5-a4 34. Rc1-d1 b7-b6 35. Rd1-d2 Rb5-c5 36. Re2-f2 Qf7-c7 37. Rd2-e2 Rf8-f5 38. Qe3-e4 Rc5-c4 39. Qe4-d3 Rc4-c5 40. Qd3-d6 Bg7xe5 41. Qd6xe6+ Qc7-f7 42. Qe6xf7+ 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 Ng8-f6 4. O-O Bf8-c5 5. c2-c3 O-O 6. d2-d4 Bc5-b6 7. Bc1-g5 h7-h6 8. Bg5-h4 d7-d6 9. Bb5xc6 b7xc6 10. Rf1-e1 Bc8-g4 11. d4xe5 d6xe5 12. Nb1-d2 g7-g5 13. Bh4-g3 Nf6-d7 14. Nd2-c4 Qd8-e7 15. a2-a4 a7-a5 16. Qd1-e2 Ra8-b8 17. Ra1-d1 f7-f6 18. h2-h3 Bg4-e6 19. Nf3-d2 Qe7-f7 20. Kg1-h2 Rf8-d8 21. b2-b4 Rb8-a8 22. Rd1-a1 a5xb4 23. c3xb4 Bb6-d4 24. Ra1-a3 Rd8-b8 25. Re1-b1 Qf7-e7 26. Nc4-e3 Bd4xe3 27. Qe2xe3 Rb8xb4 28. Rb1-a1 Ra8-a5 29. f2-f3 Nd7-b6 30. Nd2-b3 Ra5xa4 31. Qe3-c5 Qe7-d6 32. Ra3xa4 Rb4xa4 33. Ra1xa4 Nb6xa4 34. Qc5-a7 Be6xb3 35. Qa7-b8+ Kg8-g7 36. Qb8xb3 Na4-c5 37. Qb3-c4 Nc5-d3 38. h3-h4 c6-c5 39. h4-h5 Qd6-d7 40. Kh2-h1 Qd7-d6 41. Kh1-h2 Kg7-f8 42. Kh2-h1 Kf8-e7 43. Kh1-h2 Nd3-f4 44. Bg3-f2 Nf4-e6 45. Bf2-e3 Qd6-c6 46. Kh2-g3 Ke7-f7 47. Kg3-f2 Qc6-b7 48. Kf2-g3 Qb7-c6 1/2-1/2",
    "1. e2-e4 e7-e5 2. Nb1-c3 Nb8-c6 3. Bf1-c4 Ng8-f6 4. d2-d3 Bf8-b4 5. Ng1-e2 d7-d5 6. e4xd5 Nf6xd5 7. a2-a3 Bb4xc3+ 8. b2xc3 Bc8-e6 9. O-O Qd8-d7 10. Ra1-b1 b7-b6 11. Bc4xd5 Be6xd5 12. f2-f4 O-O-O 13. Qd1-e1 e5-e4 14. Qe1-f2 e4xd3 15. c2xd3 Qd7-f5 16. Qf2-g3 Bd5-c4 17. d3xc4 Qf5xb1 18. Qg3-f3 Rd8-d6 19. c4-c5 b6xc5 20. Ne2-g3 f7-f5 21. Bc1-e3 Qb1-d3 22. Rf1-c1 Rh8-e8 23. Ng3-f1 g7-g6 24. h2-h3 Re8-e4 25. Be3xc5 Rd6-d5 26. Bc5-e3 Nc6-a5 27. Qf3-f2 Na5-c4 28. Be3xa7 Re4-e2 29. Qf2-h4 Qd3-e4 30. Ba7-f2 Qe4-e7 31. Qh4-h6 Nc4-d2 32. Nf1-g3 Re2xf2 33. Kg1xf2 Qe7-c5+ 34. Kf2-e2 Nd2-c4 35. Ke2-f1 Rd5-d2 36. Ng3-h1 Nc4-e3+ 37. Kf1-e1 Rd2xg2 38. Nh1-f2 Rg2-g1+ 39. Ke1-e2 Rg1xc1 40. Qh6xh7 Rc1-c2+ 0-1",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 e7-e6 6. g2-g4 e6-e5 7. Bf1-b5+ Bc8-d7 8. Bb5xd7+ Qd8xd7 9. Nd4-f5 h7-h5 10. g4xh5 Nf6xh5 11. Nc3-d5 g7-g6 12. Nf5-g3 Bf8-e7 13. Bc1-e3 Nb8-a6 14. Qd1-d3 Na6-c7 15. Nd5xc7+ Qd7xc7 16. O-O-O a7-a6 17. Kc1-b1 Ra8-c8 18. f2-f3 Qc7-c4 19. Qd3-d2 Rc8-c6 20. Ng3-e2 Be7-d8 21. Rh1-e1 Nh5-f6 22. Ne2-g3 Qc4-b5 23. Kb1-a1 Bd8-b6 24. Be3-g5 Nf6-d7 25. c2-c3 f7-f6 26. Bg5-h6 Bb6-a5 27. Bh6-e3 Qb5-c4 28. Re1-h1 b7-b5 29. Qd2-f2 Qc4-a4 30. Rd1-c1 Ba5-b6 31. Be3xb6 Rc6xb6 32. Ka1-b1 Qa4-c4 33. h2-h4 Qc4-c5 34. Qf2-d2 Ke8-d8 35. h4-h5 g6xh5 36. Ng3xh5 Qc5-c4 37. Nh5-g7 Rh8xh1 38. Rc1xh1 Qc4-g8 39. Ng7-f5 Kd8-c7 40. Nf5-e7 Qg8-g3 41. Ne7-d5+ Kc7-c6 42. Nd5xb6 Nd7xb6 43. Rh1-d1 Nb6-c8 44. Qd2-d5+ Kc6-b6 45. Qd5-d3 Qg3-f2 46. c3-c4 Qf2-c5 47. c4xb5 Qc5xb5 48. Qd3xb5+ a6xb5 49. Rd1-h1 d6-d5 50. e4xd5 Nc8-e7 51. d5-d6 Ne7-f5 52. Rh1-d1 Nf5-d4 53. Rd1-c1 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 d7-d6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 Nb8-c6 6. Bc1-g5 e7-e6 7. Qd1-d2 Bf8-e7 8. O-O-O O-O 9. Nd4-b3 Qd8-b6 10. f2-f3 Rf8-d8 11. Bg5-e3 Qb6-c7 12. Qd2-f2 d6-d5 13. e4xd5 Nf6xd5 14. Nc3xd5 Rd8xd5 15. Rd1xd5 e6xd5 16. a2-a3 Bc8-f5 17. g2-g4 Bf5-g6 18. h2-h4 h7-h6 19. Kc1-b1 Nc6-e5 20. Nb3-d4 Ne5-c4 21. h4-h5 Bg6-h7 22. Bf1xc4 Qc7xc4 23. Rh1-d1 Ra8-c8 1/2-1/2",
    "1. e2-e4 d7-d6 2. d2-d4 Ng8-f6 3. Nb1-c3 g7-g6 4. g2-g3 Bf8-g7 5. Bf1-g2 O-O 6. Ng1-e2 e7-e5 7. h2-h3 c7-c6 8. a2-a4 Nb8-d7 9. O-O Qd8-c7 10. Bc1-e3 b7-b6 11. f2-f4 e5xd4 12. Ne2xd4 Bc8-a6 13. Rf1-e1 Ra8-d8 14. Nd4xc6 Qc7xc6 15. e4-e5 d6-d5 16. e5xf6 Nd7xf6 17. Be3-d4 Rf8-e8 18. Re1xe8+ Rd8xe8 19. Kg1-h2 Ba6-b7 20. a4-a5 Nf6-e4 21. Bd4xg7 Kg8xg7 22. Qd1-d4+ Qc6-f6 23. Nc3-b5 Qf6xd4 24. Nb5xd4 Ne4-d6 25. g3-g4 h7-h6 26. h3-h4 Re8-e7 27. Bg2-f3 Kg7-f6 28. Kh2-g3 Re7-e3 29. Kg3-f2 Re3-e8 30. a5-a6 Bb7-a8 31. Ra1-d1 Re8-c8 32. g4-g5+ h6xg5 33. h4xg5+ Kf6-e7 34. Rd1-e1+ Nd6-e4+ 35. Kf2-e3 Rc8-c5 36. Re1-h1 Ba8-c6 37. Rh1-h8 Ne4-d6 38. Rh8-b8 b6-b5 39. Bf3-e2 Ke7-d7 40. Be2-d3 Nd6-c4+ 41. Bd3xc4 1-0",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. g2-g3 Bf8-b4+ 4. Bc1-d2 Qd8-e7 5. Bf1-g2 Nb8-c6 6. Ng1-f3 Bb4xd2+ 7. Nb1xd2 d7-d6 8. O-O O-O 9. e2-e4 e6-e5 10. d4-d5 Nc6-b8 11. b2-b4 a7-a5 12. a2-a3 c7-c6 13. Nf3-e1 c6xd5 14. c4xd5 Bc8-d7 15. Ne1-c2 Nb8-a6 16. Qd1-b1 Rf8-c8 17. Nc2-e3 a5xb4 18. a3xb4 Qe7-d8 19. Qb1-b2 Bd7-b5 20. Rf1-c1 Rc8xc1+ 21. Ra1xc1 g7-g6 22. Ne3-c4 1/2-1/2",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-d2 d5xe4 4. Nd2xe4 Nb8-d7 5. Ne4-g5 e7-e6 6. Bf1-d3 Bf8-e7 7. Ng1-f3 h7-h6 8. Ng5-e4 Ng8-f6 9. c2-c3 c6-c5 10. O-O c5xd4 11. Nf3xd4 O-O 12. Rf1-e1 Rf8-e8 13. Ne4xf6+ Be7xf6 14. Bd3-c2 a7-a6 15. Qd1-g4 Kg8-h8 16. Re1-e3 Nd7-f8 17. Re3-h3 e6-e5 18. Nd4-f5 g7-g6 19. Bc1xh6 Kh8-g8 20. Rh3-g3 Qd8-b6 21. Ra1-b1 e5-e4 22. Bh6-e3 Qb6-c6 23. Qg4-f4 Bc8xf5 24. Qf4xf5 Re8-e5 25. Qf5-g4 Ra8-d8 26. Be3-f4 Re5-a5 27. Bc2-b3 Rd8-e8 28. Rb1-d1 Re8-d8 29. Rg3-e3 Rd8xd1+ 30. Qg4xd1 Nf8-e6 31. Bf4-g3 Kg8-g7 32. h2-h3 Ra5-f5 33. Bb3-c2 Ne6-c5 34. Qd1-e1 Qc6-d5 35. a2-a3 Nc5-d3 36. Qe1-e2 Qd5-b5 37. Bc2xd3 e4xd3 38. Re3xd3 Rf5-d5 39. Rd3-e3 Qb5-b3 40. Kg1-h2 Rd5-b5 41. c3-c4 1-0",
    "1. Ng1-f3 Ng8-f6 2. c2-c4 b7-b6 3. g2-g3 Bc8-b7 4. Bf1-g2 c7-c5 5. O-O g7-g6 6. d2-d3 Bf8-g7 7. e2-e4 d7-d6 8. Nb1-c3 Nb8-d7 9. h2-h3 O-O 10. Bc1-g5 Ra8-c8 11. Qd1-d2 a7-a6 12. Rf1-e1 e7-e6 13. a2-a4 Qd8-c7 14. a4-a5 b6xa5 15. Nc3-d1 Nd7-b8 16. Ra1xa5 h7-h6 17. Bg5-f4 Qc7-b6 18. Ra5-a3 Kg8-h7 19. Nd1-c3 Rf8-d8 20. Nc3-a4 Qb6-a7 21. b2-b4 Bb7-c6 22. Re1-b1 c5xb4 23. Qd2xb4 Nb8-d7 24. Qb4-a5 Nf6-e8 25. Bf4-e3 Qa7-c7 26. Na4-b6 Rc8-b8 27. Nf3-d2 Ne8-f6 28. Kg1-h2 Nd7-c5 29. e4-e5 Nc5-b7 30. Qa5xa6 d6xe5 31. Bg2xc6 Qc7xc6 32. Nb6-d5 1-0",
    "1. Ng1-f3 Ng8-f6 2. c2-c4 e7-e6 3. g2-g3 b7-b6 4. Bf1-g2 Bc8-b7 5. O-O c7-c5 6. b2-b3 Bf8-e7 7. Bc1-b2 O-O 8. d2-d3 a7-a6 9. e2-e4 d7-d6 10. Nb1-c3 Nb8-d7 11. Qd1-e2 Qd8-c7 12. Nf3-d2 Nd7-e5 13. f2-f4 Ne5-c6 14. Nd2-f3 Nf6-d7 15. Kg1-h1 Be7-f6 16. Qe2-d2 Nc6-d4 17. Nf3xd4 Bf6xd4 18. Ra1-e1 Bb7-c6 19. Bb2-a3 b6-b5 20. Nc3-e2 e6-e5 21. Ne2xd4 e5xd4 22. Kh1-g1 b5-b4 23. Ba3-c1 Qc7-b7 24. Bg2-h3 a6-a5 25. e4-e5 Qb7-c7 26. e5-e6 f7xe6 27. Bh3xe6+ Kg8-h8 28. g3-g4 a5-a4 29. g4-g5 Bc6-b7 30. Be6-g4 Rf8-f7 31. Qd2-e2 Ra8-f8 32. Bg4-f3 Bb7xf3 33. Qe2xf3 Rf7-f5 34. Re1-e6 Nd7-e5 35. f4xe5 Rf5xf3 36. Rf1xf3 Kh8-g8 37. Rf3xf8+ Kg8xf8 38. e5xd6 Qc7-d7 39. Re6-e4 Qd7xd6 40. Bc1-f4 Qd6-a6 41. b3xa4 Kf8-f7 42. Re4-e2 Qa6xa4 43. Re2-d2 Kf7-g6 44. Kg1-f2 Qa4-c6 45. Rd2-e2 Kg6-h5 46. Kf2-g3 Qc6-h1 47. Re2-d2 Qh1-f1 48. h2-h3 Qf1-h1 49. Rd2-b2 Qh1-d1 50. Rb2-d2 Qd1-f1 0-1",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Nb1-c3 Bf8-b4 4. e2-e3 b7-b6 5. Ng1-e2 Nf6-e4 6. Bc1-d2 Ne4xd2 7. Qd1xd2 Bc8-b7 8. a2-a3 Bb4xc3 9. Ne2xc3 O-O 10. Bf1-d3 Bb7xg2 11. Rh1-g1 Bg2-f3 12. e3-e4 Kg8-h8 13. Qd2-e3 Bf3-h5 14. Rg1-g5 g7-g6 15. Ke1-d2 f7-f6 16. Rg5-g3 Nb8-c6 17. Ra1-g1 e6-e5 18. d4-d5 Nc6-d4 19. Nc3-e2 c7-c5 20. Ne2xd4 e5xd4 21. Qe3-h6 Rf8-g8 22. Rg3-h3 Qd8-e7 23. f2-f4 b6-b5 24. d5-d6 Qe7-f7 25. b2-b3 Rg8-g7 26. f4-f5 Ra8-g8 27. Rg1-g2 a7-a5 28. Rg2-f2 Bh5-g4 29. Rh3-h4 g6xf5 30. h2-h3 Rg7-g5 31. e4xf5 Bg4-h5 32. Rh4-e4 Rg8-e8 33. Re4xe8+ Qf7xe8 34. Qh6xf6+ Rg5-g7 35. Rf2-g2 Qe8-f7 36. Qf6-d8+ Qf7-g8 37. Qd8xg8+ Kh8xg8 38. Rg2xg7+ Kg8xg7 39. c4xb5 Bh5-f3 40. b5-b6 Bf3-b7 41. Bd3-b5 Kg7-f6 42. Bb5xd7 Kf6-e5 43. Bd7-e8 Ke5xd6 44. f5-f6 Kd6-e6 45. f6-f7 Ke6-e7 46. a3-a4 Ke7-f8 47. Kd2-d3 h7-h6 48. Kd3-c4 Bb7-e4 49. Kc4xc5 d4-d3 50. Be8-b5 1-0",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. Nb1-c3 d5xe4 4. Nc3xe4 Nb8-d7 5. Bf1-c4 Ng8-f6 6. Ne4-g5 e7-e6 7. Qd1-e2 Nd7-b6 8. Bc4-b3 h7-h6 9. Ng5-f3 a7-a5 10. a2-a4 c6-c5 11. Bc1-f4 c5xd4 12. O-O-O Nb6-d5 13. Bf4-e5 Bf8-d6 14. Nf3xd4 O-O 15. Ng1-f3 b7-b6 16. Nd4-b5 Bc8-a6 17. Nf3-d4 Bd6xe5 18. Qe2xe5 Ba6xb5 19. Nd4xb5 Qd8-c8 20. Rd1-d4 1/2-1/2",
    "1. e2-e4 c7-c5 2. Ng1-f3 Nb8-c6 3. d2-d4 c5xd4 4. Nf3xd4 Ng8-f6 5. Nb1-c3 d7-d6 6. Bc1-g5 Qd8-a5 7. Bg5xf6 g7xf6 8. Nd4-b3 Qa5-g5 9. Nc3-d5 Ke8-d8 10. f2-f4 Qg5-h6 11. Qd1-d2 e7-e6 12. Nd5-c3 a7-a6 13. O-O-O Kd8-c7 14. g2-g3 Ra8-b8 15. Kc1-b1 Bf8-e7 16. Bf1-g2 Rh8-d8 17. Rh1-e1 Be7-f8 18. Re1-e3 b7-b6 19. Qd2-f2 Bc8-d7 20. Re3-d3 Bd7-e8 21. Nc3-e2 Kc7-b7 22. Nb3-d2 Qh6-h5 23. g3-g4 Qh5xg4 24. Bg2-f3 Qg4-g7 25. Rd3-b3 Kb7-c7 26. Nd2-c4 b6-b5 27. Nc4-e3 Rd8-c8 28. Ne2-g3 Rb8-b7 29. c2-c4 Rb7-b8 30. Rd1-c1 Nc6-a5 31. c4xb5+ Kc7-d8 32. Rb3-c3 Rb8xb5 33. Ne3-d1 f6-f5 34. e4-e5 d6-d5 35. Ng3-h5 Qg7-h6 36. Bf3-e2 Rb5-b7 37. Rc3xc8+ 1-0",
    "1. d2-d4 Ng8-f6 2. Ng1-f3 e7-e6 3. g2-g3 b7-b6 4. Bf1-g2 Bc8-b7 5. O-O c7-c5 6. b2-b3 Bf8-e7 7. Bc1-b2 O-O 8. e2-e3 d7-d6 9. c2-c4 Nb8-d7 10. Nb1-c3 a7-a6 11. a2-a4 Rf8-e8 12. d4-d5 e6xd5 13. Nf3-h4 Ra8-b8 14. c4xd5 Be7-f8 15. e3-e4 g7-g6 16. Rf1-e1 Bf8-g7 17. Qd1-c2 Bb7-c8 18. Bg2-f1 Nf6-g4 19. h2-h3 Ng4-f6 20. Nh4-f3 Nf6-h5 21. Kg1-g2 Nd7-e5 22. Nf3-g1 f7-f5 23. f2-f4 Ne5-f7 24. Ng1-f3 Qd8-d7 25. e4xf5 g6xf5 26. Re1xe8+ Qd7xe8 27. Ra1-e1 Qe8-d7 28. Re1-e6 Nh5-f6 29. Bf1-c4 Nf7-d8 30. Qc2xf5 Nd8xe6 31. d5xe6 Qd7-e7 32. Nf3-g5 Bc8-b7+ 33. Kg2-h2 Rb8-e8 34. Nc3-e4 Bb7xe4 35. Ng5xe4 Nf6-h5 36. Bb2xg7 1-0",
    "1. e2-e4 g7-g6 2. Nb1-c3 Bf8-g7 3. d2-d4 d7-d6 4. Bc1-e3 a7-a6 5. Qd1-d2 b7-b5 6. Bf1-d3 Nb8-d7 7. O-O-O c7-c6 8. h2-h4 Nd7-b6 9. h4-h5 Nb6-c4 10. Bd3xc4 b5xc4 11. h5-h6 Bg7-f8 12. Qd2-e2 d6-d5 13. Ng1-f3 Bc8-g4 14. Be3-g5 Qd8-d7 15. Rd1-e1 e7-e6 16. e4xd5 c6xd5 17. Qe2-e5 f7-f6 18. Bg5xf6 Bf8-d6 19. Qe5xd5 Bd6-f4+ 20. Kc1-b1 Qd7xd5 21. Nc3xd5 Bg4xf3 22. g2xf3 Ng8xf6 23. Re1xe6+ Ke8-d8 24. Re6xf6 Bf4-g5 25. Rf6-d6+ Kd8-c8 26. f3-f4 Bg5-d8 27. Rh1-e1 Ra8-a7 28. Re1-e6 Ra7-d7 29. c2-c3 Rd7xd6 30. Re6xd6 Kc8-b7 31. Nd5-e3 Bd8-c7 32. Rd6-f6 Rh8-e8 33. Rf6-f7 Re8-e4 34. Ne3-d5 1-0",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-c3 Bf8-b4 4. e4xd5 e6xd5 5. Bf1-d3 Nb8-c6 6. a2-a3 Bb4xc3+ 7. b2xc3 Ng8-f6 8. Ng1-e2 O-O 9. O-O Nc6-e7 10. Ne2-g3 Ne7-g6 11. f2-f4 Rf8-e8 12. f4-f5 Ng6-f8 13. Bc1-g5 Qd8-d6 14. Bg5xf6 Qd6xf6 15. Qd1-d2 Bc8-d7 16. Rf1-f3 h7-h6 17. Ng3-h5 Qf6-g5 18. Nh5-f4 c7-c6 19. Rf3-g3 Qg5-h4 20. Qd2-f2 Kg8-h8 21. Qf2-f3 Nf8-h7 22. Rg3-g4 Re8-e1+ 23. Ra1xe1 Qh4xe1+ 24. Bd3-f1 Bd7xf5 25. Rg4-g3 Bf5xc2 26. Qf3-g4 Ra8-g8 27. Qg4-d7 Nh7-g5 28. Qd7xb7 Ng5-e4 29. Rg3-f3 Bc2-d1 0-1",
    "1. d2-d4 Ng8-f6 2. c2-c4 e7-e6 3. Nb1-c3 Bf8-b4 4. f2-f3 d7-d5 5. a2-a3 Bb4-e7 6. e2-e4 d5xe4 7. f3xe4 e6-e5 8. d4-d5 Be7-c5 9. Bc1-g5 h7-h6 10. Bg5-h4 Bc5-d4 11. Ng1-f3 c7-c5 12. Bh4xf6 Qd8xf6 13. Nc3-b5 Qf6-b6 14. Nb5xd4 e5xd4 15. Bf1-d3 O-O 16. O-O Bc8-g4 17. Qd1-e1 Nb8-d7 18. Qe1-g3 Qb6-g6 19. b2-b4 Rf8-e8 20. Ra1-d1 b7-b6 21. e4-e5 Qg6-h5 22. Rd1-e1 Bg4xf3 23. e5-e6 Nd7-f6 24. Rf1xf3 Qh5-g5 25. Qg3-c7 f7xe6 26. d5xe6 Kg8-h8 27. Rf3-g3 Nf6-g4 28. h2-h3 Ra8-c8 29. Qc7-f7 Ng4-e5 30. Rg3xg5 Ne5xf7 31. Rg5-d5 Nf7-d8 32. e6-e7 Nd8-c6 33. Rd5-d7 c5xb4 34. a3xb4 Nc6xb4 35. Bd3-g6 Kh8-g8 36. Bg6xe8 Rc8xe8 37. Rd7xa7 Kg8-f7 38. Ra7-b7 Nb4-c2 39. Re1-f1+ Kf7-e6 40. Rb7xb6+ Ke6xe7 41. Rf1-d1 Re8-c8 42. Rb6-b7+ Ke7-f6 43. Rd1-f1+ Kf6-g6 44. Rf1-f7 Nc2-e3 45. Rf7xg7+ Kg6-f5 46. Rb7-c7 Rc8-b8 47. Rc7-b7 Rb8-c8 48. Rb7-c7 Rc8-b8 49. c4-c5 Rb8-b1+ 50. Kg1-f2 Kf5-e4 51. Rc7-e7+ Ke4-d3 52. c5-c6 Ne3-f5 53. c6-c7 Nf5xe7 54. Rg7xe7 Rb1-c1 55. g2-g4 Rc1-c2+ 56. Kf2-f3 Rc2-c6 57. h3-h4 Kd3-c2 58. g4-g5 h6xg5 59. h4xg5 d4-d3 60. g5-g6 Rc6-f6+ 61. Kf3-e4 Rf6-f8 62. g6-g7 Rf8-c8 63. Ke4-e5 d3-d2 64. Re7-d7 Rc8-e8+ 65. Ke5-f6 Re8-c8 66. Kf6-e6 1-0",
    "1. c2-c4 Ng8-f6 2. Ng1-f3 b7-b6 3. g2-g3 Bc8-b7 4. Bf1-g2 c7-c5 5. O-O g7-g6 6. d2-d3 Bf8-g7 7. e2-e4 d7-d6 8. Nb1-c3 O-O 9. h2-h3 Nb8-d7 10. Bc1-g5 h7-h6 11. Bg5-e3 Kg8-h7 12. Qd1-d2 Nd7-e5 13. Nf3-h4 e7-e6 14. f2-f4 Ne5-c6 15. Be3-f2 Qd8-e7 16. Nh4-f3 Ra8-d8 17. Ra1-e1 Qe7-c7 18. g3-g4 Nf6-d7 19. g4-g5 h6xg5 20. Nf3xg5+ Kh7-g8 21. f4-f5 Rd8-e8 22. Nc3-b5 Qc7-b8 23. Bf2-g3 Nc6-e5 24. f5xe6 f7xe6 25. Rf1xf8+ Nd7xf8 26. Re1-f1 a7-a6 27. Nb5-c3 Nf8-h7 28. Ng5-f3 Qb8-d8 29. Nc3-d1 Ne5xf3+ 30. Bg2xf3 d6-d5 31. e4xd5 e6xd5 32. Qd2-g2 Nh7-g5 33. Bf3xd5+ Bb7xd5 34. Qg2xd5+ Qd8xd5 35. c4xd5 Re8-d8 36. h3-h4 Ng5-f7 37. Nd1-c3 b6-b5 38. Nc3-e4 Rd8xd5 39. Ne4-f6+ Bg7xf6 40. Rf1xf6 Rd5xd3 41. Rf6xg6+ Kg8-h7 42. h4-h5 Nf7-h8 43. Rg6-g5 Nh8-f7 44. Rg5-g6 Nf7-h8 1/2-1/2",
    "1. e2-e4 c7-c6 2. d2-d4 d7-d5 3. e4xd5 c6xd5 4. c2-c4 Ng8-f6 5. Nb1-c3 e7-e6 6. Ng1-f3 Bf8-e7 7. c4xd5 Nf6xd5 8. Bf1-d3 Nb8-c6 9. O-O O-O 10. Rf1-e1 Be7-f6 11. Bd3-e4 Qd8-d6 12. Bc1-g5 Bf6xg5 13. Nf3xg5 Nd5-f6 14. d4-d5 e6xd5 15. Nc3xd5 Nf6xe4 16. Ng5xe4 Qd6-h6 17. Qd1-c1 Qh6xc1 18. Ra1xc1 Rf8-d8 19. Rc1-d1 Kg8-f8 20. f2-f3 Bc8-f5 1/2-1/2",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 c7-c5 4. e4xd5 Qd8xd5 5. Ng1-f3 c5xd4 6. Bf1-c4 Qd5-d6 7. O-O Ng8-f6 8. Nd2-b3 Nb8-c6 9. Nb3xd4 Nc6xd4 10. Nf3xd4 a7-a6 11. Bc4-b3 Qd6-c7 12. Qd1-f3 Bf8-d6 13. h2-h3 O-O 14. Bc1-g5 Nf6-d7 15. c2-c3 b7-b5 16. Ra1-d1 Bc8-b7 17. Qf3-g4 Nd7-c5 18. Bg5-f6 g7-g6 19. Rf1-e1 Nc5xb3 20. a2xb3 Rf8-e8 21. Rd1-d3 Bd6-h2+ 22. Kg1-f1 Qc7-f4 23. Qg4xf4 Bh2xf4 24. Nd4-e2 1/2-1/2",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 O-O 8. a2-a4 Bc8-b7 9. d2-d3 d7-d6 10. Nb1-c3 Nc6-a5 11. Bb3-a2 b5-b4 12. Nc3-e2 c7-c5 13. Ne2-g3 Ra8-b8 14. Nf3-d2 Rf8-e8 15. Nd2-c4 Na5xc4 16. Ba2xc4 Be7-f8 17. Bc1-g5 h7-h6 18. Bg5xf6 Qd8xf6 19. Re1-e3 Qf6-g6 20. Ng3-h5 d6-d5 21. e4xd5 Rb8-d8 22. Qd1-e2 Bb7-c8 23. Nh5-g3 Bc8-g4 24. Qe2-e1 f7-f5 25. f2-f3 f5-f4 26. Re3-e4 Bg4-d7 27. Ng3-f1 Qg6-f6 28. Nf1-d2 g7-g5 29. Qe1-f2 Kg8-h8 30. Re4-e2 Rd8-c8 31. Bc4-b3 g5-g4 32. Nd2-c4 Qf6-g5 33. f3xg4 Bd7xg4 34. Re2-e4 Rc8-d8 35. Ra1-e1 Rd8xd5 36. Re4xf4 e5xf4 37. Re1xe8 Kh8-g7 38. Nc4-e3 Rd5-d7 39. h2-h4 Qg5-g6 40. Re8xf8 1-0",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 c7-c5 4. e4xd5 e6xd5 5. Ng1-f3 c5-c4 6. b2-b3 c4xb3 7. a2xb3 Bf8-b4 8. Bf1-e2 Nb8-c6 9. O-O Ng8-e7 10. Bc1-b2 f7-f6 11. Nf3-e1 O-O 12. Ne1-d3 Bb4-d6 13. Rf1-e1 Bc8-f5 14. Nd2-f1 b7-b6 15. Nf1-g3 Bf5-g6 16. Be2-f3 a7-a5 17. Ng3-h5 Nc6-b4 18. Nd3xb4 a5xb4 19. Ra1xa8 Qd8xa8 20. Bb2-c1 Qa8-b8 21. Nh5-f4 Bg6-f7 22. g2-g3 Qb8-b7 23. Nf4-g2 Bf7-g6 24. Bc1-f4 Qb7-d7 25. Qd1-e2 Rf8-e8 26. Bf4xd6 Qd7xd6 27. Ng2-f4 Bg6-f7 28. Qe2-b5 Re8-c8 29. Bf3-g4 Rc8xc2 30. Re1-e6 Qd6-d8 31. Re6xb6 f6-f5 32. Bg4-d1 Rc2-c1 33. Rb6-b8 Ne7-c8 34. Kg1-g2 Bf7-e8 35. Qb5-b7 Be8-f7 36. Bd1-f3 Rc1-c7 37. Qb7xb4 1-0",
    "1. e2-e4 e7-e5 2. Bf1-c4 Ng8-f6 3. d2-d3 Nb8-c6 4. Ng1-f3 Bf8-c5 5. O-O d7-d6 6. c2-c3 O-O 7. a2-a4 a7-a6 8. Nb1-d2 Bc5-a7 9. Rf1-e1 Nf6-g4 10. Re1-e2 Kg8-h8 11. h2-h3 Ng4-h6 12. Nd2-f1 f7-f5 13. e4xf5 Nh6xf5 14. Bc1-g5 Qd8-e8 15. d3-d4 Qe8-g6 16. d4-d5 e5-e4 17. Re2xe4 Nc6-e5 18. Nf1-d2 Nf5-g3 19. Re4-f4 Bc8xh3 20. g2xh3 Rf8xf4 21. Bg5xf4 Ng3-e2+ 22. Kg1-h2 Ne2xf4 23. Qd1-g1 Qg6-f5 24. Bc4-f1 Nf4xd5 25. Qg1-g3 Ra8-f8 26. Ra1-e1 g7-g6 27. Bf1-g2 Ne5-d3 28. Re1-e2 Nd5-f4 29. Re2-e7 Ba7xf2 30. Qg3-g5 Nf4xg2 31. Qg5xf5 Rf8xf5 32. Kh2xg2 Bf2-b6 33. Nd2-c4 Nd3-f4+ 34. Kg2-g3 Nf4-d5 35. Re7-d7 Bb6-a7 36. Rd7-d8+ Kh8-g7 37. Rd8-a8 Ba7-c5 38. b2-b4 Nd5-b6 39. Nc4xb6 Bc5xb6 40. Ra8-b8 a6-a5 41. Rb8xb7 Rf5-d5 42. c3-c4 Rd5-f5 43. b4-b5 Rf5-f8 44. Nf3-d2 Rf8-f7 45. Nd2-e4 Bb6-d4 46. Ne4-g5 Rf7-e7 47. Rb7xc7 Re7xc7 48. Ng5-e6+ Kg7-f6 49. Ne6xc7 Kf6-e5 50. Nc7-d5 1/2-1/2",
    "1. e2-e4 Ng8-f6 2. e4-e5 Nf6-d5 3. d2-d4 d7-d6 4. Ng1-f3 g7-g6 5. Bf1-c4 Nd5-b6 6. Bc4-b3 Bf8-g7 7. a2-a4 a7-a5 8. O-O O-O 9. h2-h3 d6-d5 10. Nb1-c3 c7-c6 11. Rf1-e1 Nb8-a6 12. Nc3-e2 Na6-c7 13. c2-c3 Nb6-c4 14. Bb3-c2 b7-b5 15. b2-b3 Nc4-b6 16. Ne2-f4 b5xa4 17. b3xa4 Nb6-c4 18. Ra1-b1 f7-f6 19. e5-e6 Qd8-d6 20. Qd1-d3 f6-f5 21. Nf3-g5 Bc8-a6 22. Qd3-g3 c6-c5 23. Ng5xh7 1-0",
    "1. e2-e4 c7-c5 2. Ng1-f3 Nb8-c6 3. d2-d4 c5xd4 4. Nf3xd4 Qd8-c7 5. Nb1-c3 e7-e6 6. g2-g3 a7-a6 7. Bf1-g2 Ng8-f6 8. O-O Bf8-e7 9. Rf1-e1 Nc6xd4 10. Qd1xd4 Be7-c5 11. Qd4-d1 d7-d6 12. Bc1-e3 1/2-1/2",
    "1. e2-e4 e7-e5 2. Bf1-c4 Ng8-f6 3. d2-d3 c7-c6 4. Qd1-e2 Bf8-e7 5. Ng1-f3 d7-d6 6. O-O O-O 7. a2-a4 Qd8-c7 8. Nb1-c3 Bc8-g4 9. h2-h3 Bg4-h5 10. g2-g4 Bh5-g6 11. Nf3-h4 Kg8-h8 12. Kg1-h1 d6-d5 13. e4xd5 c6xd5 14. Bc4xd5 Nb8-c6 15. Nh4xg6+ f7xg6 16. Bd5-g2 Nc6-d4 17. Nc3-b5 Nd4xe2 18. Nb5xc7 Ra8-c8 19. Nc7-e6 Rf8-e8 20. c2-c3 Ne2xc1 21. Ra1xc1 b7-b6 22. Bg2-b7 Rc8-b8 23. Bb7-c6 Re8-c8 24. Bc6-b5 h7-h6 25. Rc1-e1 Be7-d6 26. f2-f4 e5xf4 27. Ne6xf4 g6-g5 28. Nf4-e6 Rb8-a8 29. Bb5-c4 Rc8-e8 30. Kh1-g2 a7-a6 31. Re1-e2 Re8-e7 32. Rf1-e1 Re7-b7 33. Re1-a1 Rb7-e7 34. Ra1-f1 Ra8-e8 35. Rf1-e1 Re8-a8 36. Ne6-d4 Re7xe2+ 37. Re1xe2 b6-b5 38. a4xb5 a6xb5 39. Nd4xb5 Bd6-f4 40. Nb5-d4 1-0",
    "1. e2-e4 e7-e5 2. Ng1-f3 Nb8-c6 3. Bf1-b5 a7-a6 4. Bb5-a4 Ng8-f6 5. O-O Bf8-e7 6. Rf1-e1 b7-b5 7. Ba4-b3 d7-d6 8. c2-c3 O-O 9. h2-h3 Bc8-e6 10. d2-d4 Be6xb3 11. a2xb3 e5xd4 12. c3xd4 d6-d5 13. e4-e5 Nf6-e4 14. Nb1-d2 f7-f5 15. e5xf6 Be7xf6 16. Nd2xe4 d5xe4 17. Re1xe4 Qd8-d5 18. Re4-f4 Ra8-d8 19. Bc1-e3 Nc6-b4 20. Ra1-a3 c7-c5 21. Qd1-b1 c5xd4 22. Nf3xd4 Bf6-g5 23. Rf4xf8+ Rd8xf8 24. Be3xg5 Qd5xg5 25. Nd4-e6 Qg5-f5 26. Qb1xf5 Rf8xf5 27. Ra3-a1 Rf5-e5 28. Ne6-c7 Re5-e2 29. Nc7xa6 1/2-1/2",
    "1. e2-e4 c7-c5 2. Ng1-f3 e7-e6 3. d2-d4 c5xd4 4. Nf3xd4 Nb8-c6 5. Nb1-c3 Qd8-c7 6. f2-f4 a7-a6 7. Nd4-f3 b7-b5 8. Bf1-d3 Bc8-b7 9. O-O Ng8-f6 10. e4-e5 b5-b4 11. e5xf6 b4xc3 12. b2xc3 g7xf6 13. Ra1-b1 Bf8-c5+ 14. Kg1-h1 h7-h5 15. Qd1-e1 h5-h4 16. Nf3xh4 Nc6-e7 17. Nh4-f3 Ra8-b8 18. c3-c4 Rh8-h5 19. Rb1-b3 Bb7-c6 20. Qe1-g3 Ke8-f8 21. Bc1-b2 Rh5-h6 22. Nf3-e5 Bc6-a4 23. Ne5-g4 1-0",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-c3 Bf8-b4 4. e4-e5 Ng8-e7 5. Bc1-d2 c7-c5 6. Nc3-b5 Bb4xd2+ 7. Qd1xd2 O-O 8. c2-c3 Bc8-d7 9. Ng1-f3 Qd8-b6 10. Nb5-a3 Nb8-c6 11. Bf1-e2 c5xd4 12. c3xd4 Ne7-f5 13. O-O-O f7-f6 14. e5xf6 Rf8xf6 15. Kc1-b1 Bd7-e8 16. h2-h4 Be8-h5 17. Na3-c2 a7-a5 18. Qd2-g5 Bh5xf3 19. Be2xf3 a5-a4 20. Qg5-d2 a4-a3 21. b2-b3 Ra8-a4 22. Kb1-a1 Nf5xd4 23. Nc2xd4 Ra4xd4 24. Qd2-e3 Rd4xd1+ 25. Rh1xd1 Qb6xe3 26. f2xe3 Nc6-e5 27. Rd1-f1 Ne5xf3 28. g2xf3 Rf6-g6 29. Ka1-b1 Rg6-g2 0-1",
    "1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-c3 Bf8-b4 4. e4-e5 Ng8-e7 5. Ng1-f3 c7-c5 6. d4xc5 Nb8-c6 7. Bf1-d3 Ne7-g6 8. Bd3xg6 f7xg6 9. Bc1-e3 O-O 10. O-O Bb4xc3 11. b2xc3 Qd8-c7 12. Ra1-b1 Nc6-a5 13. Rf1-e1 Bc8-d7 14. Be3-d4 Rf8-f4 15. g2-g3 Rf4-f8 16. Nf3-d2 g6-g5 17. Qd1-g4 Rf8-f5 18. Nd2-f1 Na5-c4 19. Nf1-e3 Nc4xe3 20. Bd4xe3 h7-h6 21. h2-h4 g5xh4 22. Qg4xh4 Bd7-c6 23. Rb1-b4 Qc7-f7 24. Qh4-g4 d5-d4 25. Rb4xd4 Rf5-h5 26. Rd4-e4 Ra8-d8 27. f2-f3 Bc6xe4 28. Qg4xe4 Qf7-f5 29. Qe4xf5 Rh5xf5 30. f3-f4 h6-h5 31. Re1-b1 Rf5-f7 32. Kg1-g2 Kg8-h7 33. Rb1-h1 Kh7-g6 34. Kg2-f3 Rf7-d7 35. Be3-d4 Rd8-c8 36. Rh1-h4 Rc8-c6 37. g3-g4 h5xg4+ 38. Rh4xg4+ Kg6-f7 39. Kf3-e4 Rd7-d8 40. f4-f5 e6xf5+ 41. Ke4xf5 Rd8-f8 42. Rg4-g1 Rc6-a6 43. Kf5-e4 Rf8-e8 44. Rg1-f1+ Kf7-g8 45. Rf1-a1 Re8-e7 46. a2-a4 Kg8-f7 47. a4-a5 Kf7-e8 48. Ke4-d5 Re7-d7+ 1/2-1/2",
    "1. Nb1-c3 d7-d5 2. d2-d4 Ng8-f6 3. Bc1-g5 Nb8-d7 4. Ng1-f3 h7-h6 5. Bg5-h4 e7-e6 6. e2-e3 c7-c5 7. Bf1-d3 Bf8-e7 8. O-O O-O 9. Rf1-e1 b7-b6 10. e3-e4 d5xe4 11. Nc3xe4 Bc8-b7 12. d4xc5 Nd7xc5 13. Ne4xc5 b6xc5 14. Qd1-e2 Ra8-b8 15. b2-b3 1/2-1/2",
    "1. Nb1-c3 d7-d5 2. d2-d4 Ng8-f6 3. Bc1-g5 Nb8-d7 4. Ng1-f3 h7-h6 5. Bg5-h4 e7-e6 6. e2-e3 c7-c5 7. Bf1-d3 Bf8-e7 8. O-O O-O 9. Rf1-e1 b7-b6 10. e3-e4 d5xe4 11. Nc3xe4 Bc8-b7 12. d4xc5 Nd7xc5 13. Ne4xc5 b6xc5 14. Qd1-e2 Ra8-b8 15. b2-b3 1/2-1/2"};

extern bool chessAnterior[10][10];
unsigned long timeSculpture = 0;

double lastspeedbyUser = 0;
float driverMicroSteps = -1;

hw_timer_t *timer1H = NULL;
hw_timer_t *timer2H = NULL;

volatile unsigned long timerMagnets = 0;

const int filas = 10;
const int columnas = 10;

int totalPuntosEnCurva = 90;
float distanciaEntrePuntos = 0.5;

char movChess[7][250];
char matrizObjetivo[10][10];

bool reorderChessboard = false;

// String newData = "";

int muxesOut[5] = {0, mux16Out_1, mux16Out_2, mux16Out_3, mux16Out_4};
int dirMux[10][10];

bool matrizBinSc[10][10];
bool sensorUpdate[10][10];

void sculptureMain()
{
    int cuentapatternsToHome = 0;
    char matrizPlusSculpture[10][10];
    //---------------------------------------- TURN ON ROUTINE ----------------------------------------//
    initMatrizPlus(matrizPlusSculpture);
    BleChess.setMode(1);
    if (testModeSculpture == false) // Normal mode
    {
        Serial.printf("\n \n=======================SCULPTURE MAIN======================= \n \n");
        centrarPiezasIniSc(0, matrizPlusSculpture);
        compareMatrixVsSensorsPlus(COMPARE_IMPRIME | COMPARE_CENTRA, matrizPlusSculpture);
        BleChess.sendMatrixToApp("CLEAN: Match.", matrizBinSc, matrizPlusSculpture);
    }
    else // Test mode
    {
        Serial.printf("\n \n======================SCULPTURE MAIN TEST MODE======================= \n \n");
        timeSculpture = millis();
    }
    //---------------------------------------- TURN ON ROUTINE ----------------------------------------//

    do // Loop to play the next game
    {
        int cuentaMovimientos = 0;
        if (BleChess.gameToPlay(0) == -1)
        {
            while (BleChess.gameToPlay(0) == -1 && BleChess.getModeChess() == 1)
            {
                // if the user presses play in the app and there are no games in now playing, send pause again and an error message
                if (BleChess.getPauseInfo() == 0)
                {
                    BleChess.setPauseInfo(1);
                    Serial.println("No Games in Now Playing");
                    BleChess.setState("No Games in Now Playing");
                    BleChess.sendTestModeError("No Games to play, swipe left in anygame in the library to add games to now playing.");
                }
            }
        }

        int retry = 0;
        int gameToPlay = BleChess.gameToPlay(0);
        String fullMoves = readFromFileSc(gameToPlay); // Read the file for the current game
        while (fullMoves == "EMPTY" && retry < 3)
        {
            fullMoves = readFromFileSc(gameToPlay);
            retry++;
            delay(1000);
        }
        if (fullMoves == "EMPTY")
        {
            Serial.println("Error reading file");
            BleChess.sendTestModeError("ERROR SC-MAIN-280\nSend error code to Phantom Team please.");
            BleChess.setPauseInfo(1);
            esp_restart();
        }
        int movementsinGame = (fullMoves.length());

        int previousPause = BleChess.getPauseInfo();
        BleChess.setState(previousPause == 1 ? "Paused" : "Running");
        do
        {
            int moveType = 0; // -1 dont move, 0 move back, 1 move next
            int pauseInfo = BleChess.getPauseInfo();

            if (previousPause != pauseInfo) // used to send the pause state only once
            {
                BleChess.setState(pauseInfo == 1 ? "Paused" : "Running");
                previousPause = pauseInfo;
            }

            if (gameToPlay != BleChess.gameToPlay(0) || BleChess.getModeChess() != 1) // if the game or mode changed
            {
                break;
            }

            if (pauseInfo != 1) // if not paused, in non-autoplayback mode it is useful to pause the game when it reaches the last move
            {
                moveType = 1;
                if (cuentaMovimientos != 0) // first move ins done inmediatley
                {
                    int timeToMove = BleChess.getTimeToMove() * 1000; // multiplied by 1000 to convert to milliseconds
                    unsigned long timeToMoveStart = millis();
                    bool getOutofWhile = false;
                    while (millis() - timeToMoveStart < timeToMove) // wait for the move time; other exit conditions could be added
                    {
                        if (gameToPlay != BleChess.gameToPlay(0) || BleChess.getModeChess() != 1) // if the game or mode changed
                        {
                            getOutofWhile = true;
                            break;
                        }
                    }
                    if (getOutofWhile) // if the game or mode changed
                    {
                        break;
                    }
                }
            }
            else
            {
                moveType = BleChess.getSingleMove();
                // moveType = (singleMove == 1) ? 1 : (singleMove == 0) ? 2
                //                                                    : 0;

                if (cuentaMovimientos >= movementsinGame) // if the game has already ended
                {
                    if (moveType == 1)
                    {
                        moveType = -1;
                        BleChess.sendTestModeError("No more moves next, game finished, move back or change game.");
                    }
                }

                if (cuentaMovimientos == 0)
                {
                    if (moveType == 0)
                    {
                        moveType = -1;
                        BleChess.sendTestModeError("No more previous moves, game start, move next, press play or change game.");
                    }
                }
            }

            if (moveType == 1)
            {
                movimientoSimplificado(fullMoves[cuentaMovimientos] - '0', fullMoves[cuentaMovimientos + 1] - '0', fullMoves[cuentaMovimientos + 2] - '0', fullMoves[cuentaMovimientos + 3] - '0', matrizPlusSculpture);
                if (fullMoves[cuentaMovimientos + 4] == '/')
                {
                    cuentaMovimientos += 5;
                }
                else
                {
                    movimientoSimplificado(fullMoves[cuentaMovimientos + 4] - '0', fullMoves[cuentaMovimientos + 5] - '0', fullMoves[cuentaMovimientos + 6] - '0', fullMoves[cuentaMovimientos + 7] - '0', matrizPlusSculpture);
                    cuentaMovimientos += 9;
                }
            }
            else if (moveType == 0)
            {
                movimientoSimplificado(fullMoves[cuentaMovimientos - 3] - '0', fullMoves[cuentaMovimientos - 2] - '0', fullMoves[cuentaMovimientos - 5] - '0', fullMoves[cuentaMovimientos - 4] - '0', matrizPlusSculpture);
                if (fullMoves[cuentaMovimientos - 6] == '/' || cuentaMovimientos - 6 <= 0) // condition added for when rewinding to the beginning
                {
                    cuentaMovimientos -= 5;
                }
                else
                {
                    movimientoSimplificado(fullMoves[cuentaMovimientos - 7] - '0', fullMoves[cuentaMovimientos - 6] - '0', fullMoves[cuentaMovimientos - 9] - '0', fullMoves[cuentaMovimientos - 8] - '0', matrizPlusSculpture);
                    cuentaMovimientos -= 9;
                }
            }

            if (cuentaMovimientos >= movementsinGame)
            {
                if (BleChess.getAutoPlayback())
                {
                    break;
                }
                else
                {
                    BleChess.setPauseInfo(1);
                }
            }

        } while (true); //
        fullMoves = "";
        Serial.println("-----------------------------------PREPARANDO TABLERO");

        BleChess.setState("Setting Up");

        reorderChessboardPlus(0, nullptr, matrizPlusSculpture);
        int patternsToHomeInt = BleChess.getpatternsToHome();

        if (cuentaMovimientos >= (int)(movementsinGame * 0.85)) //
        {
            cuentapatternsToHome++; // First game done. Increment the game counter.
        }

        if (cuentapatternsToHome >= patternsToHomeInt)
        {
            mechanicalCalibration(BleChess.getCalibType());
            centrarPiezasIniSc(0, matrizPlusSculpture);
            cuentapatternsToHome = 0;
        }

        if (gameToPlay == BleChess.gameToPlay(0))
        {
            Serial.println("FIN NANTURAL DEL JUEGO");
            BleChess.gameToPlay(1);
        }

        BleChess.setPauseInfo(0);
        Serial.println("-----------------------------------PREPARANDO TABLERO TERMINA");
    } while (BleChess.getModeChess() == 1);
}

void centrarPiezasIniSc(int mode, char matrixToCenter[10][10])
{ // 0 initializes the board
    // 1 uses the board in memory
    // 2 tries to fix the problem with specific pieces: if a piece is present but the sensor does not detect it, it nudges it.
    int numPoints = 0;
    int numPointsFinal = 0;
    int outPos = 8;
    int distanciaCentrado = 1; // must be non-zero; otherwise a memory error occurs.
    bool sensorsMatrix[10][10];

    BleChess.setState("Setting Up");
    Serial.println("-----------------------------------CENTRADO");
    if (mode == 0)
    {
        initMatrizPlus(matrixToCenter);
    }

    detectChessBoard(sensorsMatrix);

    // Used only for printing
    char matrizAux[10][10];
    for (int j = 0; j < 10; j++)
    {
        for (int i = 0; i < 10; i++)
        {
            if ((matrixToCenter[i][j] != '.' && (mode == 0 || mode == 1)) || (matrixToCenter[i][j] != '.' && sensorsMatrix[i][j] && mode == 2))
            {
                matrizAux[i][j] = matrixToCenter[i][j];
            }
            else
            {
                matrizAux[i][j] = '.';
            }
        }
    }
    printMatrizGenerica(matrizAux, 10, 10);

    for (int j = 0; j < 10; j++)
    {
        int iStart = (j % 2 == 0) ? 0 : 9;
        int iEnd = (j % 2 == 0) ? 10 : -1;
        int iStep = (j % 2 == 0) ? 1 : -1;
        for (int i = iStart; i != iEnd; i += iStep)
        {
            if ((matrixToCenter[i][j] != '.' && (mode == 0 || mode == 1)) || (matrixToCenter[i][j] != '.' && sensorsMatrix[i][j] && mode == 2))
            {
                int coordXBase = (50 * i) - 225;
                int coordYBase = (-50 * j) + 225;

                int coordXEnd = coordXBase;
                int coordYEnd = coordYBase;

                // ================first trajectory================
                if (i >= 1 && i <= 8 && j >= 1 && j <= 8) // Inside the board
                {
                    coordXEnd -= outPos;
                    coordYEnd += outPos;
                }
                else if (i == 0) // left graveyard
                {
                    coordYEnd -= outPos - 2;
                }
                else if (i == 9) // right graveyard
                {
                    coordYEnd += outPos + 2;
                }
                else if (j == 0) // top graveyard
                {
                    coordXEnd += outPos;
                    coordYEnd -= outPos;
                }
                else if (j == 9) // bottom graveyard
                {
                    coordXEnd += outPos;
                    coordYEnd += outPos;
                }

                float **trayectoriaAux = interpolatePoints(coordXBase, coordYBase, coordXEnd, coordYEnd, numPointsFinal);
                // ================first trajectory================

                // ================second trajectory================
                int coordXInit = coordXEnd;
                int coordYInit = coordYEnd;

                coordXEnd = coordXBase;
                coordYEnd = coordYBase;

                if (i >= 1 && i <= 8 && j >= 1 && j <= 8) // Inside the board
                {
                    coordXEnd += distanciaCentrado;
                    coordYEnd -= distanciaCentrado;
                }
                else if (i == 0) // left graveyard
                {
                    coordYEnd += distanciaCentrado + 2;
                }
                else if (i == 9) // right graveyard
                {
                    coordYEnd -= distanciaCentrado - 2;
                }
                else if (j == 0) // top graveyard
                {
                    coordXEnd -= distanciaCentrado;
                    coordYEnd += distanciaCentrado;
                }
                else if (j == 9) // bottom graveyard
                {
                    coordXEnd -= distanciaCentrado;
                    coordYEnd -= distanciaCentrado;
                }
                float **interpolatedPoints = interpolatePoints(coordXInit, coordYInit, coordXEnd, coordYEnd, numPoints);

                // ================second trajectory================

                //==================combine vectors===================
                int totalPoints = numPointsFinal + numPoints;

                float **combinedPoints = new float *[totalPoints];
                for (int i = 0; i < totalPoints; i++)
                {
                    combinedPoints[i] = new float[2];
                }

                for (int i = 0; i < numPointsFinal; i++) // Fill the combined vector with the first trajectory.
                {
                    combinedPoints[i][0] = trayectoriaAux[i][0];
                    combinedPoints[i][1] = trayectoriaAux[i][1];
                }
                for (int i = 0; i < numPoints; i++) // Fill the combined vector with the second trajectory.
                {
                    combinedPoints[i + numPointsFinal][0] = interpolatedPoints[i][0];
                    combinedPoints[i + numPointsFinal][1] = interpolatedPoints[i][1];
                }
                //==================combine vectors===================

                if (trayectoriaAux != nullptr)
                {
                    for (int i = 0; i < numPointsFinal; i++)
                    {
                        delete[] trayectoriaAux[i];
                    }
                    delete[] trayectoriaAux;
                    trayectoriaAux = nullptr; // Avoid dangling pointer
                }
                if (interpolatedPoints != nullptr)
                {
                    for (int i = 0; i < numPoints; i++)
                    {
                        delete[] interpolatedPoints[i];
                    }
                    delete[] interpolatedPoints;
                    interpolatedPoints = nullptr; // Avoid dangling pointer
                }
                // Call the function with the combined vector
                accelRampV3(combinedPoints, totalPoints, 100);

                if (combinedPoints != nullptr)
                {
                    for (int i = 0; i < totalPoints; i++)
                    {
                        delete[] combinedPoints[i];
                    }
                    delete[] combinedPoints;
                    combinedPoints = nullptr;
                }
            }

            //----------------------------------------BLUETOOTH----------------------------------------//
            // int newData = 0;
            while (BleChess.getModeChess() == 1 && BleChess.getPauseInfo() == 1)
            {
            }
            if (BleChess.getModeChess() != 1)
            {
                Serial.println("----------------------Centrado Interrumpido");
                BleChess.setState("Running");
                Serial.println("----------Change Mode Centrar");
                return;
            }
            //----------------------------------------BLUETOOTH----------------------------------------//
        }
    }
    desfaseSensado();
    BleChess.setState(BleChess.getPauseInfo() == 1 ? "Paused" : "Running");
    Serial.println("-----------------------------------CENTRADO TERMINA");
}

void reorderChessboardPlus(int modo, char matrizObjetivoAux[10][10], char matrixToReorder[10][10])
{
    // mode 0: Reorder to the initial position
    // mode 1: Reorder white pieces
    // mode 2: Reorder black pieces
    // mode 3: Reorder to a custom position
    reorderChessboard = true;

    // Matrices are indexed as [++row][++column]
    Serial.println("-----------------------------------REORDENAMIENTO AUTOMATICO");

    // GENERATE TARGET MATRIX
    char vectChessW[10] = {'R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R'};
    char vectChessB[10] = {'r', 'n', 'b', 'q', 'k', 'b', 'n', 'r'};
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            matrizObjetivo[i][j] = '.';
        }
    }
    if (modo == 0) // Reorder to the initial position
    {
        for (int i = 1; i < 9; i++)
        {
            matrizObjetivo[i][1] = vectChessB[i - 1];
        }
        for (int i = 1; i < 9; i++)
        {
            matrizObjetivo[i][2] = 'p';
        }

        for (int i = 1; i < 9; i++)
        {
            matrizObjetivo[i][8] = vectChessW[i - 1];
        }
        for (int i = 1; i < 9; i++)
        {
            matrizObjetivo[i][7] = 'P';
        }
    }
    else // Reorder to the custom position
    {
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                matrizObjetivo[i][j] = matrizObjetivoAux[i][j];
            }
        }
    }

    // MASK PIECES THAT DON'T NEED TO MOVE
    for (int j = 0; j < 10; j++) // column
    {
        for (int i = 0; i < 10; i++) // row
        {
            if (matrizObjetivo[i][j] == matrixToReorder[i][j] && matrixToReorder[i][j] != '.')
            {
                matrizObjetivo[i][j] = 'L';
            }
        }
    }

    Serial.println("Matriz Actual Inicial: ");
    printMatrizGenerica(matrixToReorder, 10, 10);

    Serial.println("Matriz Objetivo Inicial: ");
    printMatrizGenerica(matrizObjetivo, 10, 10);
    // Detect initial and final positions

    for (int expColObj = 0; expColObj < 10; expColObj++) // Explore columns of the target matrix
    {
        for (int expFilObj = 0; expFilObj < 10; expFilObj++) // Explore rows of the target matrix
        {
            detectChessBoard(matrizBinSc);
            for (int i = 0; i < 10; i++)
            {
                for (int j = 0; j < 10; j++)
                {
                    if (matrixToReorder[j][i] == '.' && matrizObjetivo[j][i] != 'L' && matrizObjetivo[j][i] != '.' && !matrizBinSc[j][i]) // If the current matrix has no piece, the target matrix has a piece, and the sensor detects the user placed something.
                    {
                        matrizObjetivo[j][i] = 'L';  // mark this space so it is not searched again.
                        matrixToReorder[j][i] = 'L'; // mark this space so it is not searched again.
                    }
                }
            }

            if (matrizObjetivo[expFilObj][expColObj] != '.' && matrizObjetivo[expFilObj][expColObj] != 'L') // If there is a piece to place in the target matrix and it is not already in position
            {
                // Serial.println("==============================================================NEXT REORDER");
                // Serial.println("Matriz Objetivo: ");
                // printMatrizGenerica(matrizObjetivo, 10, 10);
                // Serial.println("Matriz Actual: ");
                // printMatrizGenerica(matrixToReorder, 10, 10);
                // Look for its match in matrizPlus
                // Serial.print("1. Next missing piece in target matrix: ");
                // Serial.println(matrizObjetivo[expFilObj][expColObj]);

                int flagPushOutStyle = 0;
                int posfinalEstorboL = 0;
                int posfinalEstorboK = 0;

                int piezaCercanaInt = buscarPosicionCercanaPieza(expFilObj, expColObj, matrizObjetivo[expFilObj][expColObj], matrixToReorder);
                int piezaCercaObjFila = piezaCercanaInt / 10; // Get the row
                int piezaCercaObjCol = piezaCercanaInt % 10;  // Get the column
                if (piezaCercanaInt != -1)
                {
                    if (matrixToReorder[expFilObj][expColObj] != '.' || matrizBinSc[expFilObj][expColObj] == 0) // If the destination is not empty, there is a blocking piece
                    {
                        // Serial.println("2. There is a blocking piece at the target position");
                        for (int l = 0; l < 10 && flagPushOutStyle != 1; l++) // Explore columns of the target matrix
                        {
                            for (int k = 0; k < 10 && flagPushOutStyle != 1; k++) // Explore rows of the target matrix
                            {
                                if (matrizObjetivo[k][l] == matrixToReorder[expFilObj][expColObj] && (matrixToReorder[k][l] == '.' || matrizBinSc[k][l] == 1)) // If the blocking piece is needed elsewhere in the target matrix and that position is empty
                                {
                                    flagPushOutStyle = 1;
                                    posfinalEstorboK = k;
                                    posfinalEstorboL = l;
                                }
                            }
                        }
                        if (flagPushOutStyle == 1) // If the blocking piece is needed elsewhere and that position is empty
                        {
                            // Serial.println("2.1 Moving SECONDARY piece to its TARGET");
                            movimientoSimplificado(expFilObj, expColObj, posfinalEstorboK, posfinalEstorboL, matrixToReorder);
                            matrizObjetivo[posfinalEstorboK][posfinalEstorboL] = 'L'; // mark this space so it is not searched again.

                            // The destination should now be empty, so move the piece
                            // Serial.println("2.1.1 Moving PRIMARY piece to its TARGET");
                            movimientoSimplificado(piezaCercaObjFila, piezaCercaObjCol, expFilObj, expColObj, matrixToReorder);
                            matrizObjetivo[expFilObj][expColObj] = 'L'; // mark this space so it is not searched again.
                        }
                        else
                        {
                            /* First we need to move the blocking piece */
                            int posVacia = buscarPosicionCercanaVacia(expFilObj, expColObj, matrixToReorder); // Returns an integer value
                            int posVaciaFila = posVacia / 10;                                                 // Get the row
                            int posVaciaCol = posVacia % 10;                                                  // Get the column

                            // Serial.println("2.2 Moving SECONDARY piece to empty space");
                            movimientoSimplificado(expFilObj, expColObj, posVaciaFila, posVaciaCol, matrixToReorder); // Move the blocking piece; returns when it has been removed

                            // The destination should now be empty, so move the piece
                            // Serial.println("2.2.1 Moving PRIMARY piece to its TARGET");
                            movimientoSimplificado(piezaCercaObjFila, piezaCercaObjCol, expFilObj, expColObj, matrixToReorder); // move the piece to its target position
                            matrizObjetivo[expFilObj][expColObj] = 'L';                                                         // mark this space so it is not searched again.
                        }
                    }
                    else
                    {

                        // Serial.println("3. Moving PRIMARY piece to its TARGET");
                        movimientoSimplificado(piezaCercaObjFila, piezaCercaObjCol, expFilObj, expColObj, matrixToReorder); // move the piece to its target position
                        matrizObjetivo[expFilObj][expColObj] = 'L';
                        // mark these spaces so they are not searched again.
                    }
                }
                else
                {
                    // Serial.println("4. EXCEPTION: no piece found, skipping");
                    matrixToReorder[expFilObj][expColObj] = matrizObjetivo[expFilObj][expColObj];
                }
            }
        }
    }
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            matrizObjetivo[i][j] = '.';
            matrixToReorder[i][j] = '.';
        }
    }
    if (modo == 0)
    {
        if (modo == 0) // Reorder to the initial position
        {
            for (int i = 1; i < 9; i++)
            {
                matrixToReorder[i][1] = vectChessB[i - 1];
            }
            for (int i = 1; i < 9; i++)
            {
                matrixToReorder[i][2] = 'p';
            }

            for (int i = 1; i < 9; i++)
            {
                matrixToReorder[i][8] = vectChessW[i - 1];
            }
            for (int i = 1; i < 9; i++)
            {
                matrixToReorder[i][7] = 'P';
            }
        }
    }
    else
    {
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                matrixToReorder[i][j] = matrizObjetivoAux[i][j];
            }
        }
    }

    reorderChessboard = false;
    Serial.println("-----------------------------------REORDENAMIENTO AUTOMATICO TERMINA");
}

int compareMatrixVsSensorsPlus(int modo, char matrixToCompare[10][10])
{

    /* ======================SETUP===================== */
    static String newMode = "1";
    int piezasOutofPos = 0;
    int ciclosHastaSalir = 0;
    int difSensMatrix = 0;
    bool matrizPlusSensors[10][10];
    bool matrizBinScAux[10][10];
    /* ======================SETUP===================== */

    // Convert matrizPlus from char to bool: '.' or 'v' = true.
    for (int j = 0; j < 10; j++) // column
    {
        for (int i = 0; i < 10; i++) // row
        {
            matrizPlusSensors[i][j] = matrixToCompare[i][j] == '.';
        }
    }

    unsigned long tiempoInicio = millis();
    if (modo >= 0)
    {

        do // Until sensors match the virtual matrix
        {
            piezasOutofPos = 0;
            ciclosHastaSalir++;
            detectChessBoard(matrizBinSc); // Updates as many times as needed until done.

            // CHECK IF ANY PIECE IS MISPLACED OR IF THE MATRIX CHANGED
            for (int j = 0; j < 10; j++) // column
            {
                for (int i = 0; i < 10; i++) // row
                {
                    piezasOutofPos += (matrizPlusSensors[i][j] != matrizBinSc[i][j]); // increment the counter of out-of-position pieces if the virtual and sensor matrices do not match
                    difSensMatrix += (matrizBinScAux[i][j] != matrizBinSc[i][j]);     // increment the counter of piece movements if the current sensor matrix differs from the previous one
                    matrizBinScAux[i][j] = matrizBinSc[i][j];                         // update previous sensor matrix
                }
            }

            if ((ciclosHastaSalir == 1 || ciclosHastaSalir == 2 || ciclosHastaSalir == 3 || difSensMatrix > 0 || millis() - tiempoInicio >= 10000) && (modo & COMPARE_IMPRIME))
            {
                Serial.println();
                if (piezasOutofPos == 0)
                {
                    Serial.println("Chessboard and sensor matrix match.");
                }
                else
                {
                    Serial.print("ERROR: Chessboard and sensor matrix");

                    if (ciclosHastaSalir == 1 && (modo & COMPARE_CENTRA))
                    {
                        Serial.println(" do not match. Centering pieces affected...");
                        centrarPiezasIniSc(2, matrixToCompare);
                    }
                    else if (ciclosHastaSalir == 2 && (modo & COMPARE_HOME))
                    {
                        Serial.println(" still do not match. Recalibrating...");
                        mechanicalCalibration(BleChess.getCalibType());
                        Serial.printf("Calibration Finished \n Robot Initialized \n Robot Configured, Centering pieces affected...");
                        centrarPiezasIniSc(2, matrixToCompare);
                    }
                    else if (ciclosHastaSalir >= 3)
                    {
                        Serial.println(" still do not match. Please check the board and try again.");
                        BleChess.sendMatrixToApp("ERROR: Chessboard and sensor matrix do not match.", matrizBinSc, matrixToCompare);
                    }
                }

                printMatrizGenerica(matrixToCompare, 10, 10);
                Serial.println("");
                for (int j = 0; j < 10; j++)
                {
                    for (int i = 0; i < 10; i++)
                    {
                        Serial.print(matrizBinSc[i][j]);
                        Serial.print(" ");
                    }
                    Serial.println("");
                }
                Serial.println("");

                difSensMatrix = 0;
                tiempoInicio = millis(); // Reset the counter
            }
            newMode = BleChess.getModeChess();
        } while (piezasOutofPos > 0 && (modo & COMPARE_IMPRIME) && newMode.toInt() == 1); // If pieces are out of position, stay in the loop until they are fixed or the game mode changes.
    }
    else
    {
        do // Until sensors match the virtual matrix
        {
            piezasOutofPos = 0;
            ciclosHastaSalir++;
            detectChessBoard(matrizBinSc); // Updates as many times as needed until done.
            /* ======================CHECK IF ANY PIECE IS MISPLACED OR IF THE MATRIX CHANGED===================== */
            for (int j = 0; j < 10; j++) // column
            {
                for (int i = 0; i < 10; i++) // row
                {
                    piezasOutofPos += (matrizPlusSensors[i][j] != matrizBinSc[i][j]); // increment the counter of out-of-position pieces if the virtual and sensor matrices do not match
                    difSensMatrix += (matrizBinScAux[i][j] != matrizBinSc[i][j]);     // increment the counter of piece movements if the current sensor matrix differs from the previous one
                    matrizBinScAux[i][j] = matrizBinSc[i][j];                         // update previous sensor matrix
                }
            }
            /* ======================CHECK IF ANY PIECE IS MISPLACED OR IF THE MATRIX CHANGED===================== */

            if (piezasOutofPos == 0)
            {
                Serial.println("Chessboard and sensor matrix match.");
            }
            else if (piezasOutofPos > 0)
            {
                // PLAY SOUND EVERY 5 SECONDS, BUT SEND ERROR ONLY WHEN THE MATRIX CHANGES
                if (millis() - tiempoInicio >= 5000 || ciclosHastaSalir == 1)
                {
                    soundHandler(6);
                    Serial.println("ERROR: Chessboard and sensor matrix do not match. Wait for user to center.");
                    tiempoInicio = millis();
                }
                if (difSensMatrix > 0 || ciclosHastaSalir == 1)
                {
                    BleChess.sendMatrixToApp("ERROR: Chessboard and sensor matrix do not match.", matrizBinSc, matrixToCompare);
                }
            }
            difSensMatrix = 0;

            newMode = BleChess.getModeChess();
            if (globalConnect == "0")
            {
                BleChess.setMode(3);
                return -1;
                break;
            }
            // when corrected, play the success sound
            if (ciclosHastaSalir > 1 && piezasOutofPos == 0)
            {
                soundHandler(5);
            }
        } while (piezasOutofPos > 0 && newMode.toInt() == 2 && globalConnect == "1"); // If pieces are out of position, stay in the loop until they are fixed or the game mode changes.
    }
    return piezasOutofPos;
}

void movimientoSimplificado(int escaqueFilaInit, int escaqueColInit, int escaqueFilaEnd, int escaqueColEnd, char matrixToMove[10][10])
{
    int initialMode = BleChess.getModeChess();
    if (escaqueFilaInit == escaqueFilaEnd && escaqueColInit == escaqueColEnd)
    {
        String error = "ERROR SC-MOV-1. Movement From: " + String(escaqueFilaInit) + String(escaqueColInit) + (" To: ") + String(escaqueFilaEnd) + String(escaqueColEnd) + (" Modo: ") + String(initialMode) + ". Send error code to Phantom Team please.";
        Serial.println("ERROR: Same position. No movement." + error);
        BleChess.sendTestModeError(error);
        // Block the program
        /*         while (true)
                {
                    delay(1000); // Wait one second before repeating the error message
                    Serial.println("ERROR: Same position. No movement." + error);
                    BleChess.sendTestModeError(error);
                } */
        return;
    }
    else if (escaqueFilaInit < 0 || escaqueFilaInit > 9 || escaqueColInit < 0 || escaqueColInit > 9 || escaqueFilaEnd < 0 || escaqueFilaEnd > 9 || escaqueColEnd < 0 || escaqueColEnd > 9)
    {
        String error = "ERROR SC-MOV-2. Movement From: " + String(escaqueFilaInit) + String(escaqueColInit) + (" To: ") + String(escaqueFilaEnd) + String(escaqueColEnd) + (" Modo: ") + String(initialMode) + ". Send error code to Phantom Team please.";
        Serial.println("ERROR: Out of bounds. No movement." + error);
        BleChess.sendTestModeError(error);
        // Block the program
        /*         while (true)
                {
                    delay(1000); // Wait one second before repeating the error message
                    Serial.println("ERROR: Out of bounds. No movement." + error);
                    BleChess.sendTestModeError(error);
                } */
        return;
    }

    int intentos = 0;
    int intentosObjetivo = 0;
    int numPointsFinal = 0;
    bool pieceArrived = false;

    intentosObjetivo = (initialMode == 1) ? 5 : 2;
    if (testModeSculpture == true)
    {
        intentosObjetivo = 1;
        Serial.println("Test Mode");
    }
    else
    {
        Serial.println((initialMode == 1) ? "Sculpture Mode" : "Play Mode");
    }

    unsigned long timerlocal = millis();
    while (!pieceArrived && intentos < intentosObjetivo && BleChess.getModeChess() == initialMode) // && BleChess.getModeChess() == initialMode) // Try to move the mechanism intentosObjetivo times
    {
        if (intentos > 0)
        {
            Serial.print("ERROR: Piece not detected at final Position. Try number: ");
            Serial.println(intentos);
        }

        // printf("\nMoving X%d Y%d -> X%d Y%d\n", escaqueFilaInit, escaqueColInit, escaqueFilaEnd, escaqueColEnd);

        float **trayectoriaFinal = generarTrayectoria(escaqueFilaInit, escaqueColInit, escaqueFilaEnd, escaqueColEnd, numPointsFinal);
        accelRampV3(trayectoriaFinal, numPointsFinal, BleChess.getMechanismSpeed());
        if (trayectoriaFinal != nullptr)
        {
            for (int i = 0; i < numPointsFinal; i++)
            {
                delete[] trayectoriaFinal[i];
            }
            delete[] trayectoriaFinal;
            trayectoriaFinal = nullptr;
        }
        // desfaseSensado();

        detectChessBoard(matrizBinSc);
        // printf("\nSensor start: %d end: %d\n", matrizBinSc[escaqueFilaInit][escaqueColInit], matrizBinSc[escaqueFilaEnd][escaqueColEnd]);
        pieceArrived = (matrizBinSc[escaqueFilaInit][escaqueColInit] && !matrizBinSc[escaqueFilaEnd][escaqueColEnd]) ? true : false;
        intentos++;
    }

    bool waitforPiece = (initialMode == 1 || (initialMode == 2 && reorderChessboard == false)) ? true : false;
    if (intentos > intentosObjetivo && BleChess.getModeChess() == initialMode) //&& BleChess.getModeChess() == initialMode)
    {
        if (testModeSculpture == false) // normal
        {
            while (!pieceArrived && waitforPiece) //( BleChess.getModeChess() == "1" || (BleChess.getModeChess() == "2" && reorderChessboard == false))//(pieceArrived == false && BleChess.getModeChess() == "1") || (BleChess.getModeChess() == "2" && reorderChessboard == false) when reordering in play mode
            {
                if (millis() - timerlocal > 10000)
                {
                    Serial.println("ERROR: Too many attempts. Mechanism Disengage move the piece manually.");
                    String mensaje = "ERROR1: (" + String(escaqueFilaInit) + String(escaqueColInit) + String(escaqueFilaEnd) + String(escaqueColEnd) + ")";
                    BleChess.sendMatrixToApp(mensaje, matrizBinSc, matrixToMove);
                    Serial.println();
                    timerlocal = millis();
                }
                detectChessBoard(matrizBinSc);
                pieceArrived = (matrizBinSc[escaqueFilaInit][escaqueColInit] && !matrizBinSc[escaqueFilaEnd][escaqueColEnd]) ? true : false;
            }
        }
        else // testmode
        {
            deactivateAllMagnets();
            Serial.println("ERROR TEST MODE: Too many attempts" + String(escaqueFilaInit) + String(escaqueColInit) + String(escaqueFilaEnd) + String(escaqueColEnd));
            BleChess.sendTestModeError("ERROR TEST MODE: Too many attempts" + String(escaqueFilaInit) + String(escaqueColInit) + String(escaqueFilaEnd) + String(escaqueColEnd));
            unsigned long timerlocal = millis();
            while (true)
            {
                cute._tone(NOTE_G3, 50, 100);
                if (millis() - timerlocal > 10000)
                {
                    Serial.println("ERROR TEST MODE: Too many attempts" + String(escaqueFilaInit) + String(escaqueColInit) + String(escaqueFilaEnd) + String(escaqueColEnd));
                    BleChess.sendTestModeError("ERROR TEST MODE: Too many attempts" + String(escaqueFilaInit) + String(escaqueColInit) + String(escaqueFilaEnd) + String(escaqueColEnd));
                    timerlocal = millis();
                }
            }
        }
    }

    matrixToMove[escaqueFilaEnd][escaqueColEnd] = matrixToMove[escaqueFilaInit][escaqueColInit];
    matrixToMove[escaqueFilaInit][escaqueColInit] = '.';

    chessAnterior[escaqueFilaEnd][escaqueColEnd] = false;
    chessAnterior[escaqueFilaInit][escaqueColInit] = true;

    if (testModeSculpture == false) // Normal mode
    {
        if (BleChess.getModeChess() == 1)
        {
            compareMatrixVsSensorsPlus(COMPARE_IMPRIME | COMPARE_HOME | COMPARE_CENTRA, matrixToMove);
            BleChess.sendMatrixToApp("CLEAN: Match.", matrizBinSc, matrixToMove);
        }
    }
    else // Test Mode
    {
        deactivateAllMagnets();

        unsigned long elapsedTime = millis() - timeSculpture;
        unsigned long minutes = (elapsedTime / 60000) % 60; // Convert milliseconds to minutes
        unsigned long seconds = (elapsedTime / 1000) % 60;  // Convert milliseconds to seconds
        Serial.printf("Elapsed time: %lu min %lu sec\n", minutes, seconds);
        Serial.println("Piece moved successfully.");
        Serial.println();

        if (millis() - timeSculpture > 600000) // 600000) // 10 minutes = 600000
        {
            Serial.println("✓ Sculpture Mode Test Finished");
            BleChess.setMode(4);
            return;
        }
    }
}

//==============================================================MOVEMENT=======================================================
void accelRampV3(float **trayectoriaFinal, int numPointsFinal, double setSpeedbyUser)
{
    configAndCurrentPosManager(setSpeedbyUser, driverMicroSteps);
    //===============================================DELETE TRAJECTORY DUPLICATES===============================================
    int size = numPointsFinal;
    int removedCount = 0;
    for (int i = 0; i < size; i++)
    {
        for (int j = i + 1; j < size;)
        {
            if (trayectoriaFinal[i][0] == trayectoriaFinal[j][0] && trayectoriaFinal[i][1] == trayectoriaFinal[j][1])
            {
                // Shift all subsequent elements one position forward
                for (int k = j; k < size - 1; k++)
                {
                    trayectoriaFinal[k][0] = trayectoriaFinal[k + 1][0];
                    trayectoriaFinal[k][1] = trayectoriaFinal[k + 1][1];
                }
                size--;         // Reduce the array size because a duplicate was removed
                removedCount++; // Increment the count of removed elements
            }
            else
            {
                j++; // Only increment j if no element was removed, to avoid skipping checks
            }
        }
    }
    numPointsFinal = numPointsFinal - removedCount;

    //===============================================DELETE TRAJECTORY DUPLICATES===============================================
    double **speedperStep = nullptr;
    int **numberofSteps = nullptr;
    double *timeToPosition = new double[numPointsFinal]; // Allocate space for numPointsFinal elements of type double

    numberofSteps = new int *[numPointsFinal];
    for (int i = 0; i < numPointsFinal; i++)
    {
        numberofSteps[i] = new int[2];
    }

    speedperStep = new double *[numPointsFinal];
    for (int i = 0; i < numPointsFinal; i++)
    {
        speedperStep[i] = new double[2];
    }
    //---------------------------------------Mechanism Tranform----------------------------------------

    for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
    {
        speedperStep[i][0] = trayectoriaFinal[i][0];
        speedperStep[i][1] = trayectoriaFinal[i][1];
    }
    switch (BleChess.getBoardRotation())
    {
    case 0:
        break;
    case 1:
        for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
        {
            trayectoriaFinal[i][0] = speedperStep[i][1] * -1;
            ;
            trayectoriaFinal[i][1] = speedperStep[i][0];
            ;
        }
        break;
    case -1:
        for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
        {
            trayectoriaFinal[i][0] = speedperStep[i][0] * -1;
            trayectoriaFinal[i][1] = speedperStep[i][1] * -1;
        }
        break;
    case 2:
        for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
        {
            trayectoriaFinal[i][0] = speedperStep[i][1];
            trayectoriaFinal[i][1] = speedperStep[i][0] * -1;
        }
        break;
    default:
        for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
        {
            trayectoriaFinal[i][0] = speedperStep[i][0];
            trayectoriaFinal[i][1] = speedperStep[i][1];
        }
        break;
    }
    //---------------------------------------Mechanism Tranform----------------------------------------

    //===============================================ELECTROMAGNET SELECTION BY PATH=======================================================
    int electroimanes[5] = {0};
    bool electroimanesInit[5] = {false};
    bool electroimanesEnd[5] = {false};
    int electroimanesActivos[5] = {0};
    int contadorElectroactivos = 0;
    // Define conditions for each electromagnet [left, top, bottom, right]
    int condiciones[5][4] = {
        {},                     // Position 0 is not used, so indices match electromagnet numbers
        {-225, 197, -192, 182}, // Electromagnet 1
        {-214, 234, -152, 192}, // Electromagnet 2
        {-178, 197, -192, 228}, // Electromagnet 3
        {-214, 152, -234, 192}  // Electromagnet 4
    };

    for (int i = 0; i < numPointsFinal; i++) // Iterate over each trajectory point to check which electromagnet meets all conditions. If it does, increment that electromagnet's counter.
    {
        for (int j = 1; j <= 4; j++) // iterate over conditions for each electromagnet
        {
            if (trayectoriaFinal[i][0] >= condiciones[j][0] && trayectoriaFinal[i][1] <= condiciones[j][1] &&
                trayectoriaFinal[i][1] >= condiciones[j][2] && trayectoriaFinal[i][0] <= condiciones[j][3])
            {
                electroimanes[j]++;
                if (i == 0)
                    electroimanesInit[j] = true;
                if (i == numPointsFinal - 1)
                    electroimanesEnd[j] = true;
            }
        }
    }

    for (int i = 1; i <= 4; i++) // If an electromagnet can travel the entire trajectory, add it to the active electromagnets array
    {
        if (electroimanes[i] == numPointsFinal)
        {
            electroimanesActivos[contadorElectroactivos++] = i;
        }
    }
    //===============================================ELECTROMAGNET SELECTION BY PATH=======================================================

    //=======================================================MOVEMENT TYPE=======================================================
    float electroOffsetX = 0;
    float electroOffsetY = 0;

#ifdef MODELFINAL

    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * -(driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / -(driverMicroSteps * 5)) + currentYposition;
#endif
#ifdef MODELEFRA
    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * (driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / (driverMicroSteps * 5)) + currentYposition;
#endif

    int electroActivo = -1;
    int electroActivoInit = -1;
    int electroActivoEnd = -1;

    double minDistance = std::numeric_limits<double>::max();
    //--------------------------------------------------Multiple Electromagnets Movement-------------------------------------------------------
    if (contadorElectroactivos == 0) // Complex movement
    {

        for (int i = 1; i <= 4; i++) // Find the electromagnet closest to the start point
        {
            if (electroimanes[i] < minDistance && electroimanesInit[i] == true)
            {
                minDistance = electroimanes[i];
                electroActivoInit = i;
            }
        }

        minDistance = std::numeric_limits<double>::max();
        for (int i = 1; i <= 4; i++) // Find the electromagnet closest to the end point
        {
            if (electroimanes[i] < minDistance && electroimanesEnd[i] == true)
            {
                minDistance = electroimanes[i];
                electroActivoEnd = i;
            }
        }

        calcularOffsets(electroActivoInit, electroOffsetX, electroOffsetY);
        for (int i = 0; i < (numPointsFinal - electroimanes[electroActivoEnd]); i++) // Apply offset to all points in trajectory 1 and inverse kinematics
        {
            trayectoriaFinal[i][0] = trayectoriaFinal[i][0] + electroOffsetX;
            trayectoriaFinal[i][1] = trayectoriaFinal[i][1] + electroOffsetY;
        }

        calcularOffsets(electroActivoEnd, electroOffsetX, electroOffsetY);
        for (int i = (numPointsFinal - electroimanes[electroActivoEnd]); i < (numPointsFinal); i++) // Apply offset to all points in trajectory 2 and inverse kinematics
        {
            trayectoriaFinal[i][0] = trayectoriaFinal[i][0] + electroOffsetX;
            trayectoriaFinal[i][1] = trayectoriaFinal[i][1] + electroOffsetY;
        }
    }
    //--------------------------------------------------Multiple Electromagnets Movement-------------------------------------------------------

    //--------------------------------------------------One Electromagnets Movement-------------------------------------------------------
    else // Simple movement
    {

        for (int i = 0; i < contadorElectroactivos; i++) // Find the electromagnet closest to the start point
        {
            double deltaX, deltaY, distance;

            calcularOffsets(electroimanesActivos[i], electroOffsetX, electroOffsetY);
            deltaX = (trayectoriaFinal[0][0] + electroOffsetX) - currentXposition;
            deltaY = (trayectoriaFinal[0][1] + electroOffsetY) - currentYposition;
            distance = sqrt(deltaX * deltaX + deltaY * deltaY); // Calculate the distance between the start point and the electromagnet

            if (distance < minDistance) // If the distance is less than the minimum distance, update the active electromagnet
            {
                minDistance = distance;
                electroActivo = electroimanesActivos[i];
            }
        }

        calcularOffsets(electroActivo, electroOffsetX, electroOffsetY);
        for (int i = 0; i < numPointsFinal; i++) // Apply offset to all trajectory points and inverse kinematics
        {
            trayectoriaFinal[i][0] = trayectoriaFinal[i][0] + electroOffsetX;
            trayectoriaFinal[i][1] = trayectoriaFinal[i][1] + electroOffsetY;
        }
    }
    //--------------------------------------------------One Electromagnets Movement-------------------------------------------------------
    //==========================================================MOVEMENT TYPE=======================================================

    //=======================================================INVERSE KINEMATICS=======================================================
    for (int i = 0; i < numPointsFinal; i++)
    {
#ifdef MODELFINAL

        numberofSteps[i][0] = round(-(driverMicroSteps * 5) * (trayectoriaFinal[i][0] - trayectoriaFinal[i][1]));
        numberofSteps[i][1] = round(-(driverMicroSteps * 5) * (-trayectoriaFinal[i][0] - trayectoriaFinal[i][1]));
#endif
#ifdef MODELEFRA
        numberofSteps[i][0] = round((driverMicroSteps * 5) * (trayectoriaFinal[i][0] - trayectoriaFinal[i][1]));
        numberofSteps[i][1] = round((driverMicroSteps * 5) * (-trayectoriaFinal[i][0] - trayectoriaFinal[i][1]));
#endif
    }
    //=======================================================INVERSE KINEMATICS=======================================================

    //=====================================================SPEED AND ACCELERATION=========================================================
    float umbralInferior = 0.52;                    // Approximately 30 degrees in radians
    float umbralSuperior = 2 * PI - umbralInferior; // Approximately 330 degrees in radians
    std::vector<int> puntosCambioDireccion;

    //--------------------------------------------------Calculates Speed-------------------------------------------------------
    for (int i = 0; i < numPointsFinal; i++) // Iterate over each trajectory point and assign a constant speed to each element
    {
        double deltaX = trayectoriaFinal[i][0] - ((i == 0) ? currentXposition : trayectoriaFinal[i - 1][0]);
        double deltaY = trayectoriaFinal[i][1] - ((i == 0) ? currentYposition : trayectoriaFinal[i - 1][1]);

        double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
        timeToPosition[i] = distance / setSpeedbyUser; // The integer part of the division is the time to reach the position

        double currentPositionInStepsX = (i == 0) ? stepper1.currentPosition() : numberofSteps[i - 1][0];
        double currentPositionInStepsY = (i == 0) ? stepper2.currentPosition() : numberofSteps[i - 1][1];

        speedperStep[i][0] = (timeToPosition[i] == 0) ? 0 : ((numberofSteps[i][0] - currentPositionInStepsX) / timeToPosition[i]); // assign speed to the speed matrix
        speedperStep[i][1] = (timeToPosition[i] == 0) ? 0 : ((numberofSteps[i][1] - currentPositionInStepsY) / timeToPosition[i]); // assign speed to the speed matrix

        if ((i > 1 && i < numPointsFinal - 1)) // Direction change; protected against adding duplicate points
        {
            double dirActual = atan2(trayectoriaFinal[i][1] - trayectoriaFinal[i - 1][1], trayectoriaFinal[i][0] - trayectoriaFinal[i - 1][0]);
            double dirSiguiente = atan2(trayectoriaFinal[i + 1][1] - trayectoriaFinal[i][1], trayectoriaFinal[i + 1][0] - trayectoriaFinal[i][0]);
            double cambioDir = abs(dirSiguiente - dirActual);

            if (cambioDir >= umbralInferior && cambioDir <= umbralSuperior)
            {
                puntosCambioDireccion.push_back(i);
            }
        }
        // printf("Punto %d Distance: %f Time: %f X: %f Y: %f PosM1: %d PosM2: %d SpeedM1: %f SpeedM2: %f\n",
        //    i, distance, timeToPosition[i], trayectoriaFinal[i][0], trayectoriaFinal[i][1],
        //  numberofSteps[i][0], numberofSteps[i][1], speedperStep[i][0], speedperStep[i][1]);
    }
    //--------------------------------------------------Calculates Speed-------------------------------------------------------
    puntosCambioDireccion.push_back(numPointsFinal - 1); // Add the last trajectory point (index-based, not data count)

    //--------------------------------------------------Calculates Accelararion-------------------------------------------------------
    for (int i = 0; i < puntosCambioDireccion.size(); i++) // Accelerations per segment according to direction changes; here we know the number of segments.
    {
        double segmentTime = 0;
        double timeX = 0;
        double y1, y2, y, a1, a2, c1, c2;

        int inicioSegmento = (i == 0) ? 1 : puntosCambioDireccion[i - 1] + 1; // Segment start
        int finSegmento = puntosCambioDireccion[i];                           // Segment end

        for (int i = inicioSegmento; i <= finSegmento; ++i) // Calculate the time for each segment
        {
            segmentTime = (i == inicioSegmento) ? 0 : segmentTime + timeToPosition[i];
        }

        if (segmentTime < 0.25)
        {
            a1 = 75;
            a2 = 75;
            c1 = 0.03;
        }
        else
        {
            a1 = 35;
            a2 = 35;
            c1 = 0.005;

            float Y_objetivo = 5000;
            float Y_calculado;
            bool encontrado = false;

            while (!encontrado) // Search for a dynamic c1 value.
            {
                double deltaSpeedM1 = abs(speedperStep[inicioSegmento + 2][0] - speedperStep[inicioSegmento + 1][0]);
                double deltaSpeedM2 = abs(speedperStep[inicioSegmento + 2][1] - speedperStep[inicioSegmento + 1][1]);

                if (deltaSpeedM1 > deltaSpeedM2) // Use the motor with the larger speed change for the calculation
                {
                    Y_calculado = speedperStep[inicioSegmento + 1][0] * (1 / (1 + exp(-a1 * (-c1))));
                }
                else
                {
                    Y_calculado = speedperStep[inicioSegmento + 1][1] * (1 / (1 + exp(-a1 * (-c1))));
                }

                if (abs(Y_calculado) < Y_objetivo) // if the calculated Y meets the requirement, set the flag to exit the loop
                {
                    encontrado = true;
                }
                else // If the target is not yet reached, calculate another C1
                {
                    c1 += 0.005; // Increment C1 by 0.005 and repeat
                }
            }
        }
        c2 = segmentTime - c1; // Calculate c2 based on c1

        for (int i = inicioSegmento; i <= finSegmento; i++) // Generate s-curve accelerations for each element of the segment
        {
            timeX = timeX + ((i == inicioSegmento) ? 0 : timeToPosition[i]);

            y1 = 1 / (1 + exp(-a1 * (timeX - c1)));
            y2 = 1 / (1 + exp(-a2 * (timeX - c2)));
            y = abs((y1)-y2);

            speedperStep[i][0] = speedperStep[i][0] * y;
            speedperStep[i][1] = speedperStep[i][1] * y;
        }
    }
    //--------------------------------------------------Calculates Accelararion-------------------------------------------------------
    //================================================SPEED AND ACCELERATION=========================================================
    stepper1.setMaxSpeed(10000000); // Prevent interference with speed calculations
    stepper2.setMaxSpeed(10000000); // Prevent interference with speed calculations

    // ================================================== MOVE MOTORS =============================================================
    int umbralVelocidad = 10000;
    bool runProtection = false;
    bool graveyardMove = false;
    unsigned long timetosafety = millis();
    unsigned long timetoTurnOff = 30000; // 30 seconds to turn off the system

    if (trayectoriaFinal[numPointsFinal][0] > 200 || trayectoriaFinal[numPointsFinal][0] < -200 || trayectoriaFinal[numPointsFinal][1] > 200 || trayectoriaFinal[numPointsFinal][1] < -200)
    {
        /*  preferences.begin("myApp", false);
         String jumpToCenter = preferences.getString("jumpToCenter", "2");
         preferences.end(); */

        int jumpToCenter = BleChess.getJumpToCenter();

        if (jumpToCenter == 1)
        {
            Serial.println("=====================================================================================JALON ELECTROIMAN");
            graveyardMove = true;
        }
    }

    // Serial.println("Preciclo");
    unsigned long timeMovingLocal = millis();
    for (int i = 0; i < numPointsFinal; i++)
    {

        int stepsM1 = numberofSteps[i][0] - stepper1.currentPosition();
        int stepsM2 = numberofSteps[i][1] - stepper2.currentPosition();

        float initialPosition1 = stepper1.currentPosition();
        float initialPosition2 = stepper2.currentPosition();

        float deltaX = abs(stepsM1);
        float deltaY = abs(stepsM2);

        double stepIsDue = deltaX > deltaY ? static_cast<double>(deltaY) / deltaX : static_cast<double>(deltaX) / deltaY;
        double stepSum = 0.0;
        int dirDeGiro1 = stepsM1 > 0 ? 1 : -1;
        int dirDeGiro2 = stepsM2 > 0 ? 1 : -1;

        //--------------------------------------------Protects the motors from a sudden change in speed--------------------------------------
        if (runProtection == true)
        {
            stepper1.setMaxSpeed(10000000);
            stepper2.setMaxSpeed(10000000);
        }
        runProtection = false;
        if (i > 1) // Activate .run function if there is a very large speed change
        {
            if (speedperStep[i][1] - speedperStep[i - 1][1] > umbralVelocidad || speedperStep[i][0] - speedperStep[i - 1][0] > umbralVelocidad)
            {
                runProtection = true;
            }
        }
        //--------------------------------------------Protects the motors from a sudden change in speed--------------------------------------

        //--------------------------------------------Electromagnets activation and deactivation--------------------------------------------------
        if (i == 1) // activates electromagnet in the first cycle
        {
            activateElectromagnetV2((contadorElectroactivos == 0) ? electroActivoInit : electroActivo, 35);
            stepper1.setMaxSpeed(10000000);
            stepper2.setMaxSpeed(10000000);
        }

        if (graveyardMove == true && i == numPointsFinal - 13) // Dactivates electromagnet in the last 13 cycles so it makes a pull on the electromagnet
        {
            deactivateAllMagnets();
        }

        if (contadorElectroactivos == 0) // Mechanism is making a complex movement with a different electromagnet for each segment
        {
            if (i == (numPointsFinal - electroimanes[electroActivoEnd])) // Deactivates electroActivoInit
            {
                deactivateAllMagnets();
            }
            else if (i == (numPointsFinal - electroimanes[electroActivoEnd]) + 1) // Activates electroActivoEnd in the following cycle.
            {
                activateElectromagnetV2(electroActivoEnd, 35);
            }
        }

        if (millis() - timetosafety > timetoTurnOff) // if time is greater than 60 seconds, the electromagnet and motors are disabled
        {
            deactivateAllMagnets();
            stepper1.disableOutputs();
            stepper2.disableOutputs();
            while (true)
            {
                if (millis() - timetosafety > timetoTurnOff) // prints a message every timetoTurnOff seconds
                {
                    Serial.println("Safety Time, Electromagnet and motors disabled");
                    timetosafety = millis();
                }
            }
        }
        else
        {
            timetosafety = millis();
        }
        //--------------------------------------------Electromagnets activation and deactivation--------------------------------------------------

        // --------------------------------------------------Pure Motor Movement-------------------------------------------------------------------
        if (deltaX > deltaY)
        {
            if (i == 0 || runProtection == true)
            {
                stepper1.setAcceleration(speedperStep[i][0] * 2);
                stepper1.setMaxSpeed(speedperStep[i][0] * 1);
                stepper1.moveTo(numberofSteps[i][0]);
            }
            else
            {
                stepper1.setSpeed(speedperStep[i][0]);
                stepper2.setSpeed(speedperStep[i][1]);
            }
            while ((abs(stepper1.currentPosition() - numberofSteps[i][0]) > 0) && (millis() - timetosafety < timetoTurnOff))
            {
                if (i == 0 || runProtection == true)
                {
                    stepper1.run();
                }
                else
                {
                    stepper1.runSpeed();
                }

                if (abs(stepper1.currentPosition() - initialPosition1) >= 1)
                {
                    stepSum += stepIsDue;
                    initialPosition1 = stepper1.currentPosition();
                    while (stepSum >= 1 && (millis() - timetosafety < timetoTurnOff))
                    {
                        stepper2.setSpeed(1000000.0 * dirDeGiro2);
                        stepper2.runSpeed();
                        stepSum -= 1;
                    }
                }
            }
        }
        else
        {
            if (i == 0 || runProtection == true)
            {
                stepper2.setAcceleration(speedperStep[i][1] * 2);
                stepper2.setMaxSpeed(speedperStep[i][1] * 1);
                stepper2.moveTo(numberofSteps[i][1]);
            }
            else
            {
                stepper1.setSpeed(speedperStep[i][0]);
                stepper2.setSpeed(speedperStep[i][1]);
            }
            while ((abs(stepper2.currentPosition() - numberofSteps[i][1]) > 0) && (millis() - timetosafety < timetoTurnOff))
            {
                if (i == 0 || runProtection == true)
                {
                    stepper2.run();
                }
                else
                {
                    stepper2.runSpeed();
                }

                if (abs(stepper2.currentPosition() - initialPosition2) >= 1)
                {
                    stepSum += stepIsDue;
                    initialPosition2 = stepper2.currentPosition();
                    while (stepSum >= 1 && (millis() - timetosafety < timetoTurnOff))
                    {
                        stepper1.setSpeed(1000000.0 * dirDeGiro1);
                        stepper1.runSpeed();
                        stepSum -= 1;
                    }
                }
            }
        }
        // --------------------------------------------------Pure Motor Movement-------------------------------------------------------------------
    }
    timeMovingLocal = millis() - timeMovingLocal;
    timeMoving = timeMoving + timeMovingLocal;

    if (graveyardMove)
    {
        activateElectromagnetV2(contadorElectroactivos == 0 ? electroActivoEnd : electroActivo, 35);
        delay(200);
    }

    if (testModeSculpture == false)
    {
        deactivateAllMagnets();
    }

    // ================================================== MOVE MOTORS =============================================================

    // ================================================= FREE MEMORY ==========================================================
    if (timeToPosition != nullptr)
    {
        delete[] timeToPosition;  // Free the entire array
        timeToPosition = nullptr; // Ensure the pointer does not point to a freed location
    }

    if (speedperStep != nullptr)
    {
        for (int i = 0; i < numPointsFinal; i++)
        {
            delete[] speedperStep[i];
        }
        delete[] speedperStep;
        speedperStep = nullptr;
    }

    if (numberofSteps != nullptr)
    {
        for (int i = 0; i < numPointsFinal; i++)
        {
            delete[] numberofSteps[i];
        }
        delete[] numberofSteps;
        numberofSteps = nullptr;
    }
    // At this point, the size and capacity are probably equal.
    puntosCambioDireccion.clear();
    // Now the size is 0, but the capacity remains 1000.

    // If you really need to reduce the memory used by a vector after clearing or substantially modifying it:
    puntosCambioDireccion.shrink_to_fit();
    // ================================================= FREE MEMORY ==========================================================
    driver.rms_current(HOLD_CURRENT);
    driver2.rms_current(HOLD_CURRENT);
}

//---------------------CONTROL ELECTROMAGNERT POWER WITH PWM
/*    void activateElectromagnetV2(int optionElectro, int power)
{
    // Serial.print("Activating electromagnet: ");
    // Serial.println(optionElectro);

    // Update the activation time of the electromagnets
    timerMagnets = millis();

    // Initialize the timer if not already initialized
    if (timer2H == NULL)
    {
        timer2H = timerBegin(3, 80, true); // Prescaler 80 for 1us resolution with an 80MHz clock
        timerAttachInterrupt(timer2H, &timeEnabled, true);
        timerAlarmWrite(timer2H, 10000000, true); // 10 seconds
        timerAlarmEnable(timer2H);
        // Serial.println("==============================================Timer enabled");
    }

    const int magnetPins[4] = {magnet1, magnet2, magnet3, magnet4};

    // Turn off all electromagnets initially
    for (int i = 0; i < 4; ++i)
    {
        ledcDetachPin(magnetPins[i]); // Disconnect the PWM channel from the electromagnet pin
        digitalWrite(magnetPins[i], HIGH);
    }

    // Configure the PWM channel to stabilize power
    const int freq = 60;
    const int ledChannel = 0;
    const int resolution = 8;

    ledcSetup(ledChannel, freq, resolution);

    // Verify that the electromagnet option is valid
    if (optionElectro >= 1 && optionElectro <= 4)
    {
        // Attach the PWM channel to the selected electromagnet pin
        ledcAttachPin(magnetPins[optionElectro - 1], ledChannel);

        // Set the electromagnet power
        ledcWrite(ledChannel, power);
    }
    // Serial.println("Electromagnet activated");
}
 void deactivateAllMagnets()
{
    // Serial.println("Deactivating all electromagnets");
    const int magnetPins[4] = {magnet1, magnet2, magnet3, magnet4};

    // Disconnect all PWM channels from the electromagnet pins
    for (int i = 0; i < 4; ++i)
    {
        ledcDetachPin(magnetPins[i]);      // Disconnect the PWM channel from the electromagnet pin
        digitalWrite(magnetPins[i], HIGH); // Set the pin high (off)
    }

    // Disable and stop the timer
    if (timer2H != NULL)
    {
        timerAlarmDisable(timer2H);
        timerDetachInterrupt(timer2H);
        timerEnd(timer2H);
        timer2H = NULL; // Ensure the pointer is NULL after freeing the timer
                        //  Serial.println("==============================================Timer disabled");
    }
    // Serial.println("All electromagnets deactivated");
} */
//---------------------CONTROL ELECTROMAGNERT POWER WITH PWM

void activateElectromagnetV2(int optionElectro, int power)
{
    // Serial.print("Activating electromagnet: ");
    // Serial.println(optionElectro);

    // Update the activation time of the electromagnets
    timerMagnets = millis();

    // Initialize the timer if not already initialized
    if (timer2H == NULL)
    {
        timer2H = timerBegin(3, 80, true); // Prescaler 80 for 1us resolution with an 80MHz clock
        timerAttachInterrupt(timer2H, &timeEnabled, true);
        timerAlarmWrite(timer2H, 10000000, true); // 10 seconds
        timerAlarmEnable(timer2H);
        // Serial.println("==============================================Timer enabled");
    }

    const int magnetPins[4] = {magnet1, magnet2, magnet3, magnet4};

    // Turn off all electromagnets initially
    for (int i = 0; i < 4; ++i)
    {
        digitalWrite(magnetPins[i], HIGH); // Turn off the electromagnet
    }

    // Verify that the electromagnet option is valid
    if (optionElectro >= 1 && optionElectro <= 4)
    {
        // Activate the selected electromagnet at full power
        digitalWrite(magnetPins[optionElectro - 1], LOW);
    }
    // Serial.println("Electromagnet activated");
}

void deactivateAllMagnets()
{
    // Serial.println("Deactivating all electromagnets");
    const int magnetPins[4] = {magnet1, magnet2, magnet3, magnet4};

    // Turn off all electromagnets
    for (int i = 0; i < 4; ++i)
    {
        digitalWrite(magnetPins[i], HIGH); // Set the pin high (off)
    }

    // Disable and stop the timer
    if (timer2H != NULL)
    {
        timerAlarmDisable(timer2H);
        timerDetachInterrupt(timer2H);
        timerEnd(timer2H);
        timer2H = NULL; // Ensure the pointer is NULL after freeing the timer
                        //  Serial.println("==============================================Timer disabled");
    }
    // Serial.println("All electromagnets deactivated");
}

void IRAM_ATTR timeEnabled()
{
    // Compare elapsed time with 60000 ms (60 seconds)
    if (millis() - timerMagnets > 60000) // 60000
    {
        deactivateAllMagnets(); // Deactivate all electromagnets
        stepper1.disableOutputs();
        stepper2.disableOutputs();
    }
}
//==============================================================MOVEMENT=======================================================

//==========================================================TRAJECTORY GENERATOR======================================================
float **generarTrayectoria(int escaqueFilaInit, int escaqueColInit, int escaqueFilaEnd, int escaqueColEnd, int &numPointsFinal)
{

    int totalCurvas = 0;
    int numPointsCurva = 0;
    int numPoints = 0; // Will be updated based on the hypotenuse
    int numPointsAux = 0;
    int numPointsAux2 = 0;
    double arrayCurvas[totalPuntosEnCurva * 3][2];
    float **interpolatedPoints = nullptr;
    float **interpolatedPoints2 = nullptr;
    float **interpolatedPoints3 = nullptr;
    float **trayectoriaFinal = nullptr;
    // 0,0 = -225, 225 9,9 = 225, -225
    int coordXInit = (50 * escaqueFilaInit) - 225;
    int coordYInit = (-50 * escaqueColInit) + 225;

    int coordXEnd = (50 * escaqueFilaEnd) - 225;
    int coordYEnd = (-50 * escaqueColEnd) + 225;

    int tipodeMov = movementType(escaqueFilaInit, escaqueColInit, escaqueFilaEnd, escaqueColEnd);

    printf("%s X%d Y%d -> X%d Y%d\n", (tipodeMov == 0) ? "Directo" : "Entre Lineas", escaqueFilaInit, escaqueColInit, escaqueFilaEnd, escaqueColEnd);
    const float generalOffset = BleChess.getOffsetPieces();

    //  Serial.println("generalOffset: " + String(generalOffset));
    //   ==============================================================trajectory generators ==============================================================
    //   -------------------------------------------------------------------Direct Movement --------------------------------------------------------------------
    if (tipodeMov == 0) // direct movement
    {

        interpolatedPoints = interpolatePoints(coordXInit, coordYInit, coordXEnd, coordYEnd, numPoints); // Returns a matrix with interpolated points and also sets numPoints.

        numPointsFinal = numPoints + (generalOffset / distanciaEntrePuntos); // the +1 accounts for the value added at the end of the trajectory

        trayectoriaFinal = new float *[numPointsFinal];
        for (int i = 0; i < numPointsFinal; i++)
        {
            trayectoriaFinal[i] = new float[2];
        }
        for (int i = 0; i < numPoints; i++)
        {
            trayectoriaFinal[i][0] = interpolatedPoints[i][0];
            trayectoriaFinal[i][1] = interpolatedPoints[i][1];
        }
    }
    // -------------------------------------------------------------------Direct Movement --------------------------------------------------------------------

    // -----------------------------------------------------------------Between-Line Movement --------------------------------------------------------------------
    else if (tipodeMov == 1) // between-line movement
    {
        moveOnTheLinev2Sc(coordXInit, coordYInit, coordXEnd, coordYEnd, totalCurvas, arrayCurvas); // Returns an integer numPoints with the total number of points (16 or 24) and an ARRAYCURVAS array with the point coordinates

        numPointsCurva = totalCurvas * totalPuntosEnCurva;

        if (totalCurvas == 2) // short between-line movement with 2 curves
        {
            interpolatedPoints = interpolatePoints(arrayCurvas[totalPuntosEnCurva - 1][0], arrayCurvas[totalPuntosEnCurva - 1][1], arrayCurvas[totalPuntosEnCurva][0], arrayCurvas[totalPuntosEnCurva][1], numPoints); // get the points from the end of the first curve to the start of the second curve

            numPointsFinal = numPoints + numPointsCurva + (generalOffset / distanciaEntrePuntos); // the +1 accounts for the value added at the end of the trajectory

            trayectoriaFinal = new float *[numPointsFinal];
            for (int i = 0; i < numPointsFinal; i++)
            {
                trayectoriaFinal[i] = new float[2];
            }

            for (int i = 0; i < totalPuntosEnCurva; i++)
            {
                trayectoriaFinal[i][0] = arrayCurvas[i][0];
                trayectoriaFinal[i][1] = arrayCurvas[i][1];
            }
            for (int i = 0; i < numPoints; i++)
            {
                trayectoriaFinal[i + totalPuntosEnCurva][0] = interpolatedPoints[i][0];
                trayectoriaFinal[i + totalPuntosEnCurva][1] = interpolatedPoints[i][1];
            }
            for (int i = 0; i < totalPuntosEnCurva; i++)
            {
                trayectoriaFinal[i + totalPuntosEnCurva + numPoints][0] = arrayCurvas[i + totalPuntosEnCurva][0];
                trayectoriaFinal[i + totalPuntosEnCurva + numPoints][1] = arrayCurvas[i + totalPuntosEnCurva][1];
            }
        }

        if (totalCurvas == 3) // long between-line movement with 3 curves
        {
            interpolatedPoints = interpolatePoints(arrayCurvas[totalPuntosEnCurva - 1][0], arrayCurvas[totalPuntosEnCurva - 1][1], arrayCurvas[totalPuntosEnCurva][0], arrayCurvas[totalPuntosEnCurva][1], numPoints);                         // get the points from the end of the first curve to the start of the second curve
            interpolatedPoints2 = interpolatePoints(arrayCurvas[(totalPuntosEnCurva * 2) - 1][0], arrayCurvas[(totalPuntosEnCurva * 2) - 1][1], arrayCurvas[totalPuntosEnCurva * 2][0], arrayCurvas[totalPuntosEnCurva * 2][1], numPointsAux); // get the points from the end of the second curve to the start of the third curve

            numPointsFinal = numPoints + numPointsAux + numPointsCurva + (generalOffset / distanciaEntrePuntos); // the +1 accounts for the value added at the end of the trajectory

            trayectoriaFinal = new float *[numPointsFinal];
            for (int i = 0; i < numPointsFinal; i++)
            {
                trayectoriaFinal[i] = new float[2];
            }

            for (int i = 0; i < totalPuntosEnCurva; i++)
            {
                trayectoriaFinal[i][0] = arrayCurvas[i][0];
                trayectoriaFinal[i][1] = arrayCurvas[i][1];
            }
            for (int i = 0; i < numPoints; i++)
            {
                trayectoriaFinal[i + totalPuntosEnCurva][0] = interpolatedPoints[i][0];
                trayectoriaFinal[i + totalPuntosEnCurva][1] = interpolatedPoints[i][1];
            }
            for (int i = 0; i < totalPuntosEnCurva; i++)
            {
                trayectoriaFinal[i + totalPuntosEnCurva + numPoints][0] = arrayCurvas[i + totalPuntosEnCurva][0];
                trayectoriaFinal[i + totalPuntosEnCurva + numPoints][1] = arrayCurvas[i + totalPuntosEnCurva][1];
            }
            for (int i = 0; i < numPointsAux; i++)
            {
                trayectoriaFinal[i + totalPuntosEnCurva + numPoints + totalPuntosEnCurva][0] = interpolatedPoints2[i][0];
                trayectoriaFinal[i + totalPuntosEnCurva + numPoints + totalPuntosEnCurva][1] = interpolatedPoints2[i][1];
            }
            for (int i = 0; i < totalPuntosEnCurva; i++)
            {
                trayectoriaFinal[i + totalPuntosEnCurva + numPoints + totalPuntosEnCurva + numPointsAux][0] = arrayCurvas[i + totalPuntosEnCurva * 2][0];
                trayectoriaFinal[i + totalPuntosEnCurva + numPoints + totalPuntosEnCurva + numPointsAux][1] = arrayCurvas[i + totalPuntosEnCurva * 2][1];
            }
        }
    }
    // -----------------------------------------------------------------Between-Line Movement --------------------------------------------------------------------
    int numPointsAdded = generalOffset / distanciaEntrePuntos;       // Number of points to add at the end of the trajectory
    int lastPointIndex = numPointsFinal - numPointsAdded - 1;        // Index of the last point
    int penultimatePointIndex = numPointsFinal - numPointsAdded - 2; // Index of the second-to-last point

    float xLast = trayectoriaFinal[lastPointIndex][0];
    float yLast = trayectoriaFinal[lastPointIndex][1];
    float xPenultimate = trayectoriaFinal[penultimatePointIndex][0];
    float yPenultimate = trayectoriaFinal[penultimatePointIndex][1];

    float theta = atan2(yLast - yPenultimate, xLast - xPenultimate);

    float xNew = xLast + generalOffset * cos(theta);
    float yNew = yLast + generalOffset * sin(theta);

    if (coordXEnd > 200 || coordXEnd < -200 || coordYEnd > 200 || coordYEnd < -200)
    {
        for (int i = 0; i < numPointsAdded; i++)
        {
            trayectoriaFinal[i + numPointsFinal - numPointsAdded][0] = xLast;
            trayectoriaFinal[i + numPointsFinal - numPointsAdded][1] = yLast;
        }
    }

    else
    {
        interpolatedPoints3 = interpolatePoints(xLast, yLast, xNew, yNew, numPointsAux2); // Returns a matrix with interpolated points and also sets numPoints.

        for (int i = 0; i < numPointsAux2; i++)
        {
            trayectoriaFinal[i + numPointsFinal - numPointsAdded][0] = interpolatedPoints3[i][0];
            trayectoriaFinal[i + numPointsFinal - numPointsAdded][1] = interpolatedPoints3[i][1];
        }
    }

    // ----------------------------------------------------------centering offset compensation----------------------------------------------------------------
    if (interpolatedPoints != nullptr)
    {
        for (int i = 0; i < numPoints; i++)
        {
            delete[] interpolatedPoints[i];
        }
        delete[] interpolatedPoints;
        interpolatedPoints = nullptr; // Avoid dangling pointer
    }
    if (interpolatedPoints2 != nullptr)
    {
        for (int i = 0; i < numPointsAux; i++)
        {
            delete[] interpolatedPoints2[i];
        }
        delete[] interpolatedPoints2;
        interpolatedPoints2 = nullptr;
    }
    if (interpolatedPoints3 != nullptr)
    {
        for (int i = 0; i < numPointsAux2; i++)
        {
            delete[] interpolatedPoints3[i];
        }
        delete[] interpolatedPoints3;
        interpolatedPoints3 = nullptr;
    }

    // ==============================================================trajectory generators ==============================================================

    return trayectoriaFinal;
}

void moveOnTheLinev2Sc(double xIni, double yIni, double xFin, double yFin, int &totalCurvas, double arrayPuntosFinal[][2])
// Returns the total number of points sent.
{
    //=======Intermediate points for knight trajectory=========
    float diferenciaX;
    float diferenciaY;
    float puntoInterX;
    float puntoInterY;

    int vectInterPointsX[5] = {-1, -1, -1, -1, -1};
    int vectInterPointsY[5] = {-1, -1, -1, -1, -1};

    /*======================================TRAJECTORY CONDITIONS==========================================*/

    diferenciaX = abs(xIni - xFin); // distance traveled in X
    diferenciaY = abs(yIni - yFin); // distance traveled in Y

    vectInterPointsX[0] = xIni;
    vectInterPointsY[0] = yIni;

    if (diferenciaX > diferenciaY) // If the distance in X is greater than the distance in Y
    {
        if (yFin < yIni) // Electromagnet moving from top to bottom
        {
            puntoInterX = xIni;
            puntoInterY = yIni - 25;

            vectInterPointsX[1] = puntoInterX;
            vectInterPointsY[1] = puntoInterY;
        }
        if (yFin > yIni) // Electromagnet moving from bottom to top
        {
            puntoInterX = xIni;
            puntoInterY = yIni + 25;

            vectInterPointsX[1] = puntoInterX;
            vectInterPointsY[1] = puntoInterY;
        }

        if (xFin < xIni) // Electromagnet moving from right to left
        {
            puntoInterX = xFin + 25;

            vectInterPointsX[2] = puntoInterX;
            vectInterPointsY[2] = puntoInterY;
        }
        if (xFin > xIni) // Electromagnet moving from left to right
        {
            puntoInterX = xFin - 25;

            vectInterPointsX[2] = puntoInterX;
            vectInterPointsY[2] = puntoInterY;
        }
        puntoInterY = yFin; //

        vectInterPointsX[3] = puntoInterX;
        vectInterPointsY[3] = puntoInterY;
    }
    else // If the distance in Y is greater than the distance in X
    {
        if (xFin < xIni) // Electromagnet moving from right to left
        {
            puntoInterX = xIni - 25;
            puntoInterY = yIni;

            vectInterPointsX[1] = puntoInterX;
            vectInterPointsY[1] = puntoInterY;
        }
        if (xFin > xIni) // Electromagnet moving from left to right
        {
            puntoInterX = xIni + 25;
            puntoInterY = yIni;

            vectInterPointsX[1] = puntoInterX;
            vectInterPointsY[1] = puntoInterY;
        }
        if (yFin < yIni) // Electromagnet moving from top to bottom
        {
            puntoInterY = yFin + 25;

            vectInterPointsX[2] = puntoInterX;
            vectInterPointsY[2] = puntoInterY;
        }
        if (yFin > yIni) // Electromagnet moving from bottom to top
        {
            puntoInterY = yFin - 25;

            vectInterPointsX[2] = puntoInterX;
            vectInterPointsY[2] = puntoInterY;
        }
        puntoInterX = xFin;

        vectInterPointsX[3] = puntoInterX;
        vectInterPointsY[3] = puntoInterY;
    }

    vectInterPointsX[4] = xFin;
    vectInterPointsY[4] = yFin;

    if (diferenciaX < diferenciaY)
    {
        if (diferenciaX == 50)
        {
            vectInterPointsX[2] = vectInterPointsX[1];
            vectInterPointsY[2] = yFin;

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
        if (diferenciaX == 0)
        {
            // The case where they are in the same column of the board
            if (xIni > 0) // This condition avoids the motors
            {
                vectInterPointsX[1] = xIni - 25;
                vectInterPointsY[1] = yIni;

                vectInterPointsX[2] = vectInterPointsX[1];
                vectInterPointsY[2] = yFin;

                vectInterPointsX[3] = xFin;
                vectInterPointsY[3] = yFin;

                vectInterPointsX[4] = -1;
                vectInterPointsY[4] = -1;
            }
            else // The original implementation only included the following
            {
                vectInterPointsX[1] = xIni + 25;
                vectInterPointsY[1] = yIni;

                vectInterPointsX[2] = vectInterPointsX[1];
                vectInterPointsY[2] = yFin;

                vectInterPointsX[3] = xFin;
                vectInterPointsY[3] = yFin;

                vectInterPointsX[4] = -1;
                vectInterPointsY[4] = -1;
            }
        }
    }

    if (diferenciaX > diferenciaY)
    {
        if (diferenciaY == 50)
        {
            vectInterPointsX[2] = xFin;
            vectInterPointsY[2] = vectInterPointsY[1];

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
        if (diferenciaY == 0)
        {
            // The case where they are in the same column of the board
            if (yIni > 0) // This condition avoids the motors
            {
                vectInterPointsX[1] = xIni;
                vectInterPointsY[1] = yIni - 25;

                vectInterPointsX[2] = xFin;
                vectInterPointsY[2] = vectInterPointsY[1];

                vectInterPointsX[3] = xFin;
                vectInterPointsY[3] = yFin;

                vectInterPointsX[4] = -1;
                vectInterPointsY[4] = -1;
            }
            else // The original implementation only included the following
            {
                vectInterPointsX[1] = xIni;
                vectInterPointsY[1] = yIni + 25;

                vectInterPointsX[2] = xFin;
                vectInterPointsY[2] = vectInterPointsY[1];

                vectInterPointsX[3] = xFin;
                vectInterPointsY[3] = yFin;

                vectInterPointsX[4] = -1;
                vectInterPointsY[4] = -1;
            }
        }
    }
    if (diferenciaX == 50 && diferenciaY == 50)
    {
        if (xIni < xFin)
        {
            vectInterPointsX[1] = xIni + 25;
            vectInterPointsY[1] = yIni;

            vectInterPointsX[2] = vectInterPointsX[1];
            vectInterPointsY[2] = yFin;

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
        else
        {
            vectInterPointsX[1] = xIni - 25;
            vectInterPointsY[1] = yIni;

            vectInterPointsX[2] = vectInterPointsX[1];
            vectInterPointsY[2] = yFin;

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
    }
    /*======================================TRAJECTORY CONDITIONS==========================================*/
    double arrayPuntosX[totalPuntosEnCurva] = {0};
    double arrayPuntosY[totalPuntosEnCurva] = {0};

    if (vectInterPointsX[4] == -1 || vectInterPointsY[4] == -1) // If there is no curve endpoint
    {
        totalCurvas = 2;
        puntosDeCurvaBezier(vectInterPointsX[0], vectInterPointsY[0], vectInterPointsX[1], vectInterPointsY[1], vectInterPointsX[2], vectInterPointsY[2], arrayPuntosX, arrayPuntosY, totalPuntosEnCurva);
        for (int i = 0; i < totalPuntosEnCurva; i++)
        {
            arrayPuntosFinal[i][0] = arrayPuntosX[i];
            arrayPuntosFinal[i][1] = arrayPuntosY[i];
        }
        puntosDeCurvaBezier(vectInterPointsX[1], vectInterPointsY[1], vectInterPointsX[2], vectInterPointsY[2], vectInterPointsX[3], vectInterPointsY[3], arrayPuntosX, arrayPuntosY, totalPuntosEnCurva);
        for (int i = 0; i < totalPuntosEnCurva; i++)
        {
            arrayPuntosFinal[i + totalPuntosEnCurva][0] = arrayPuntosX[i];
            arrayPuntosFinal[i + totalPuntosEnCurva][1] = arrayPuntosY[i];
        }
    }
    else
    {
        totalCurvas = 3;
        puntosDeCurvaBezier(vectInterPointsX[0], vectInterPointsY[0], vectInterPointsX[1], vectInterPointsY[1], vectInterPointsX[2], vectInterPointsY[2], arrayPuntosX, arrayPuntosY, totalPuntosEnCurva);
        for (int i = 0; i < totalPuntosEnCurva; i++)
        {
            arrayPuntosFinal[i][0] = arrayPuntosX[i];
            arrayPuntosFinal[i][1] = arrayPuntosY[i];
        }
        puntosDeCurvaBezier(vectInterPointsX[1], vectInterPointsY[1], vectInterPointsX[2], vectInterPointsY[2], vectInterPointsX[3], vectInterPointsY[3], arrayPuntosX, arrayPuntosY, totalPuntosEnCurva);
        for (int i = 0; i < totalPuntosEnCurva; i++)
        {
            arrayPuntosFinal[i + totalPuntosEnCurva][0] = arrayPuntosX[i];
            arrayPuntosFinal[i + totalPuntosEnCurva][1] = arrayPuntosY[i];
        }
        puntosDeCurvaBezier(vectInterPointsX[2], vectInterPointsY[2], vectInterPointsX[3], vectInterPointsY[3], vectInterPointsX[4], vectInterPointsY[4], arrayPuntosX, arrayPuntosY, totalPuntosEnCurva);
        for (int i = 0; i < totalPuntosEnCurva; i++)
        {
            arrayPuntosFinal[i + totalPuntosEnCurva * 2][0] = arrayPuntosX[i];
            arrayPuntosFinal[i + totalPuntosEnCurva * 2][1] = arrayPuntosY[i];
        }
    }
}

int movementType(int escaqueFilaInit, int escaqueColInit, int escaqueFilaEnd, int escaqueColEnd) // MYBE MATRIZTOMOVEMENT IS A MUST, ITS THE CURRENT MATRIX
{
    if (escaqueFilaInit - escaqueFilaEnd == 0 || escaqueColInit - escaqueColEnd == 0 || abs(escaqueFilaInit - escaqueFilaEnd) == abs(escaqueColInit - escaqueColEnd)) // If the movement is in a straight line or diagonal
    {
        // tipodeMov = 0 -> direct movement
        // tipodeMov = 1 -> between-line movement
        if (abs(escaqueFilaInit - escaqueFilaEnd) == abs(escaqueColInit - escaqueColEnd)) // If the movement is in a diagonal
        {
            int minX = min(escaqueFilaInit, escaqueFilaEnd);
            int maxX = max(escaqueFilaInit, escaqueFilaEnd);
            int cuentaVacios = 0;
            detectChessBoard(matrizBinSc);
            for (int x = minX + 1; x < maxX; x++) // Explore the diagonal from the initial square to the final square
            {
                int y = -1;
                if (escaqueFilaEnd > escaqueFilaInit)
                {
                    if (escaqueColEnd > escaqueColInit) // bottom-right
                    {
                        y = escaqueColInit + (x - escaqueFilaInit);
                    }
                    else // top-right
                    {
                        y = escaqueColInit - (x - escaqueFilaInit);
                    }
                }
                else
                {
                    if (escaqueColEnd > escaqueColInit) // bottom-left
                    {
                        y = escaqueColInit + (escaqueFilaInit - x);
                    }
                    else // top-left
                    {
                        y = escaqueColInit - (escaqueFilaInit - x);
                    }
                }

                if (matrizBinSc[x][y] == 1) // && matriztomovement[x][y] == '.')
                {
                    cuentaVacios++;
                }
            }

            if (cuentaVacios == (maxX - minX) - 1)
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }
        else if (escaqueFilaInit - escaqueFilaEnd == 0) // If the movement is in a straight line in the Y axis
        {
            int minY = min(escaqueColInit, escaqueColEnd);
            int maxY = max(escaqueColInit, escaqueColEnd);
            int cuentaVacios = 0;
            int x = escaqueFilaInit;
            for (int y = minY + 1; y < maxY; y++)
            {

                if (matrizBinSc[x][y] == 1) // && matriztomovement[x][y] == '.')
                {
                    cuentaVacios++;
                }
            }
            if (cuentaVacios == (maxY - minY) - 1)
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }
        else if (escaqueColInit - escaqueColEnd == 0) // If the movement is in a straight line in the X axis
        {
            int minX = min(escaqueFilaInit, escaqueFilaEnd);
            int maxX = max(escaqueFilaInit, escaqueFilaEnd);
            int cuentaVacios = 0;
            int y = escaqueColEnd;
            for (int x = minX + 1; x < maxX; x++)
            {
                if (matrizBinSc[x][y] == 1) // && matriztomovement[x][y] == '.')
                {
                    cuentaVacios++;
                }
            }
            if (cuentaVacios == (maxX - minX) - 1)
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }
    }
    else // If the movement is not in a straight line or diagonal
    {
        return 1;
    }
}

float **interpolatePoints(float x1, float y1, float x2, float y2, int &numPoints)
{
    // Calculate the hypotenuse
    float hypotenuse = std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));

    // Round the number of points to the nearest integer
    numPoints = static_cast<int>(hypotenuse / distanciaEntrePuntos + 0.5f);

    // Ensure numPoints is at least 2 to avoid division by zero
    if (numPoints < 2)
    {
        numPoints = 2;
    }

    // Create a dynamic matrix on the heap
    float **interpolatedPoints = new float *[numPoints];
    for (int i = 0; i < numPoints; i++)
    {
        interpolatedPoints[i] = new float[2];
    }

    // Perform the interpolation
    for (int i = 0; i < numPoints; i++)
    {
        float t = static_cast<float>(i) / (numPoints - 1);
        interpolatedPoints[i][0] = x1 + (x2 - x1) * t;
        interpolatedPoints[i][1] = y1 + (y2 - y1) * t;
    }

    return interpolatedPoints;
}

void puntosDeCurvaBezier(double xP1, double yP1, double xP2, double yP2, double xP3, double yP3, double *arrayTestX, double *arrayTestY, int sizeArray)
{

    xP1 = (xP2 > xP1) ? xP2 - 25 : ((xP2 < xP1) ? xP2 + 25 : xP1);
    yP1 = (yP2 > yP1) ? yP2 - 25 : ((yP2 < yP1) ? yP2 + 25 : yP1);
    xP3 = (xP2 > xP3) ? xP2 - 25 : ((xP2 < xP3) ? xP2 + 25 : xP3);
    yP3 = (yP2 > yP3) ? yP2 - 25 : ((yP2 < yP3) ? yP2 + 25 : yP3);
    for (int i = 0; i < sizeArray; i++)
    {
        double t = i / (double)(sizeArray - 1); // Distribute t values uniformly between 0 and 1
        // Apply the Bézier equation to calculate points on the curve
        arrayTestX[i] = (1 - t) * (1 - t) * xP1 + 2 * (1 - t) * t * xP2 + t * t * xP3;
        arrayTestY[i] = (1 - t) * (1 - t) * yP1 + 2 * (1 - t) * t * yP2 + t * t * yP3;
    }
}

void calcularOffsets(int electroActivo, float &electroOffsetX, float &electroOffsetY)
{
    switch (electroActivo)
    {
    case 1:
        electroOffsetX = 20.58;
        electroOffsetY = 0;
        break;
    case 2:
        electroOffsetY = -39.96;
        electroOffsetX = 10.54;
        break;
    case 3:
        electroOffsetX = -25.42;
        electroOffsetY = 0;
        break;
    case 4:
        electroOffsetY = 39.96;
        electroOffsetX = 10.54;
        break;
    default:
        electroOffsetX = 0;
        electroOffsetY = 0;
        break;
    }
}

//==========================================================TRAJECTORY GENERATOR======================================================

//==========================================================GAME DECODING=====================================================
String decodificaMovimiento(char x_inichar, char y_inichar, char x_finchar, char y_finchar, char action, char piezacolor, char matrixToDecode[10][10], bool mode)
{
    int x_ini;
    int y_ini;
    int x_fin;
    int y_fin;
    int x_finaux;
    int y_finaux;
    bool flagAlPassant = false;
    bool lugarDisponible = false;

    String sendMove = "";
    bool chess_color = (piezacolor == 'W') ? true : false; // false = black, true = white

    /* CONVERSOR DE CHAR A MATRIZPLUS */
    char posCoordBoardX[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
    char posCoordBoardY[8] = {'8', '7', '6', '5', '4', '3', '2', '1'};

    // Search for coordinates to update the board matrix
    for (int i = 0; i < 8; i++) // Scan in X
    {
        if (x_inichar == posCoordBoardX[i]) // If the letter matches the letter in the array, return the position index
        {
            x_ini = i + 1;
        }
        if (x_finchar == posCoordBoardX[i]) // If the letter matches the letter in the array, return the position index
        {
            x_fin = i + 1;
        }
        if (y_inichar == posCoordBoardY[i]) // If the letter matches the letter in the array, return the position index
        {
            y_ini = i + 1;
        }
        if (y_finchar == posCoordBoardY[i]) // If the letter matches the letter in the array, return the position index
        {
            y_fin = i + 1;
        }
    }
    /* CONVERSOR DE CHAR A MATRIZPLUS */

    if (mode == true)
    {
        detectChessBoard(matrizBinSc);
    }
    else
    {
        for (int i = 0; i < 10; i++)
        {
            for (int j = 0; j < 10; j++)
            {
                matrizBinSc[i][j] = 1;
            }
        }
    }
    //----------------------------------------COMER----------------------------------------//

    if (action == '1') // if the move is a capture
    {
        chess_color = !chess_color; // Invert the color because we are decoding the opponent's capture move
        // Decode the final graveyard position.
        if (chess_color == false) // if black
        {
            if (matrixToDecode[x_fin][y_fin] == '.') // no piece to capture at the destination, probably en passant.
            {

                if (matrixToDecode[x_fin][y_fin + 1] == 'P' || matrixToDecode[x_fin][y_fin + 1] == 'p') // confirm there is a piece at the new position.
                {
                    Serial.println("Al passant");
                    flagAlPassant = true;
                    y_fin = y_fin + 1;
                }
                else
                {
                    Serial.println("No hay pieza en el destino, verificar partida.");
                }
            }
            if (matrixToDecode[x_fin][y_fin] == 'P' || matrixToDecode[x_fin][y_fin] == 'p')
            {
                for (int i = 1; i <= 8; i++) // scan for an available graveyard spot; 0 and 9 are excluded as corners. Placed ascending or descending.
                {
                    if (matrixToDecode[i][9] == '.' && matrizBinSc[i][9] == 1) // check the bottom graveyard; special black pieces go here.
                    {
                        x_finaux = i;
                        y_finaux = 9;
                        lugarDisponible = true;
                    }
                }
            }
            else
            {
                for (int i = 1; i <= 8; i++) // scan for an available graveyard spot; 0 and 9 are excluded as corners. Placed ascending or descending.
                {
                    if (matrixToDecode[9][i] == '.' && matrizBinSc[9][i] == 1) // check the right graveyard; special black pieces go here.
                    {
                        x_finaux = 9;
                        y_finaux = i;
                        lugarDisponible = true;
                    }
                }
            }
        }
        else if (chess_color == true) // if white
        {
            if (matrixToDecode[x_fin][y_fin] == '.') // no piece to capture at the destination, probably en passant.
            {

                if (matrixToDecode[x_fin][y_fin - 1] == 'P' || matrixToDecode[x_fin][y_fin - 1] == 'p') // confirm there is a piece at the new position.
                {
                    Serial.println("Al passant");
                    flagAlPassant = true;
                    y_fin = y_fin - 1;
                }
                else
                {
                    Serial.println("No hay pieza en el destino, verificar partida.");
                }
            }

            if (matrixToDecode[x_fin][y_fin] == 'P' || matrixToDecode[x_fin][y_fin] == 'p') // If it is a pawn
            {
                for (int i = 8; i >= 1; i--) // scan for an available graveyard spot; 0 and 9 are excluded as corners. Placed ascending or descending.
                {
                    if (matrixToDecode[i][0] == '.' && matrizBinSc[i][0] == 1) // check the bottom graveyard; special black pieces go here.
                    {
                        x_finaux = i;
                        y_finaux = 0;
                        lugarDisponible = true;
                    }
                }
            }
            else
            {
                for (int i = 8; i >= 1; i--) // scan for an available graveyard spot; 0 and 9 are excluded as corners. Placed ascending or descending.
                {
                    if (matrixToDecode[0][i] == '.' && matrizBinSc[0][i] == 1) // check the left graveyard; special white pieces go here.
                    {
                        x_finaux = 0;
                        y_finaux = i;
                        lugarDisponible = true;
                    }
                }
            }
        }
        if (!lugarDisponible)
        {
            // Move piece to any available empty space '.'
            for (int j = 0; j < 10; j++)
            {
                for (int i = 0; i < 10; i++)
                {
                    if (matrixToDecode[i][j] == '.' && matrizBinSc[i][j] == 1 && (i == 0 || i == 9 || j == 0 || j == 9) && !(i == 0 && j == 0) && !(i == 9 && j == 9) && !(i == 0 && j == 9) && !(i == 9 && j == 0))
                    {
                        // Serial.println("Random graveyard spot found. Moving piece there.");
                        x_finaux = i;
                        y_finaux = j;
                        lugarDisponible = true;
                    }
                }
            }
        }

        sendMove = String(x_fin) + String(y_fin) + String(x_finaux) + String(y_finaux);

        if (flagAlPassant == true) // adjust final position to the correct one, not the capture search position
        {
            if (chess_color == false)
            {
                y_fin = y_fin - 1;
            }
            else if (chess_color == true)
            {
                y_fin = y_fin + 1;
            }
        }

        sendMove = sendMove + String(x_ini) + String(y_ini) + String(x_fin) + String(y_fin);
    }
    //----------------------------------------COMER----------------------------------------//

    //----------------------------------------ENROQUES----------------------------------------//
    else if (action == '2') // Short castling
    {
        if (chess_color == true) // If it is white's turn
        {
            sendMove = "58788868";
        }
        else // If it is black's turn
        {
            sendMove = "51718161";
        }
    }
    else if (action == '3') // Long castling
    {
        if (chess_color == true) // If it is white's turn
        {
            sendMove = "58381848";
        }
        else // If it is black's turn
        {
            sendMove = "51311141";
        }
    }
    //----------------------------------------ENROQUES----------------------------------------//
    else // The move is a simple move (onlymove).
    {
        sendMove = String(x_ini) + String(y_ini) + String(x_fin) + String(y_fin);
    }

    sendMove = sendMove + "/";
    return sendMove;
}

String readFromFileSc(int gameToPlay)
{
    // Serial.println("------------------------------------------------------Reading from file");
    //  Serial.println("Reading from file");
    int index_mov = 0;
    int indexA = 0;
    int gameLenght = 0;
    char movChessInd[7];
    const char *game = "";
    char gameBuffer[2048]; // Adjust the size as needed
    // int gameToPlay = playlistIndices[cuentaPartidas];

    if (gameToPlay < 200)
    {
        gameLenght = strlen(games[gameToPlay]);
        game = games[gameToPlay];
    }
    else if (gameToPlay > 200 && gameToPlay < 300)
    {
        std::string gameToPlayAux = std::to_string(gameToPlay);
        const char *gameToPlayPref = gameToPlayAux.c_str();
        Serial.print("Game to play pref: ");
        Serial.println(gameToPlayPref);
        preferences.begin("PL_pgn");
        String gameString = preferences.getString(gameToPlayPref, "EMPTY");
        if (gameString == "EMPTY")
        {
            Serial.println("PLAYLIST READ EMPTY");
            return "EMPTY";
        }
        preferences.end();
        strncpy(gameBuffer, gameString.c_str(), sizeof(gameBuffer) - 1);
        gameBuffer[sizeof(gameBuffer) - 1] = '\0'; // Ensure null-terminated
        game = gameBuffer;
        Serial.print("Game to play: ");
        Serial.println(game);

        gameLenght = strlen(game);
        Serial.print("Game length: ");
        Serial.println(gameLenght);
    }
    else
    {
        Serial.println("INDEX OUT OF BOUNDS");
    }

    char matrixToSimulateGame[10][10];

    String fullMoves = "";

    initMatrizPlus(matrixToSimulateGame);
    // Serial.print("Initial Matrix: ");
    // printMatrizGenerica(matrixToSimulateGame, 10, 10);
    indexA = 0;
    while (indexA < gameLenght) // Iterate over the entire game string
    {
        // Serial.print("ASCII: ");
        // Serial.print(game[indexA]);
        //// print the hexadecimal value to see what is being read
        // Serial.print("      HEX: ");
        // Serial.println(game[indexA], DEC);

        if (game[indexA] == ' ') // If a space is found, a new move is starting
        {
            for (int i = 0; i < 7; i++) // initialize the entire move to V
            {
                movChessInd[i] = 'v';
            }

            indexA++;                                                                 // move to the next character
            int indexAInit = indexA;                                                  // Save the starting position of the move
            while (game[indexA] != ' ' && game[indexA] != '.' && indexA < gameLenght) // while a space, period, or end of string is not found
            {
                // Serial.print("ASCII: ");
                // Serial.print(game[indexA]);
                // // print the hexadecimal value to see what is being read
                // Serial.print("      HEX: ");
                // Serial.println(game[indexA], DEC);

                movChessInd[indexA - indexAInit] = game[indexA];
                indexA++;
            }

            if (game[indexA] == ' ')
            {
                String auxDecode = decodeChessMove(movChessInd, index_mov);
                // Serial.print("Move: ");
                // Serial.println(auxDecode);
                String localMove = decodificaMovimiento(auxDecode[2], auxDecode[3], auxDecode[4], auxDecode[5], auxDecode[1], auxDecode[0], matrixToSimulateGame, false);
                // Serial.print("Decoded move: ");
                //  Serial.println(localMove);
                // simulating the move
                matrixToSimulateGame[localMove[2] - '0'][localMove[3] - '0'] = matrixToSimulateGame[localMove[0] - '0'][localMove[1] - '0'];
                matrixToSimulateGame[localMove[0] - '0'][localMove[1] - '0'] = '.';
                // Serial.println("Matrix after move: ");
                // printMatrizGenerica(matrixToSimulateGame, 10, 10);
                if (localMove[4] != '/')
                {
                    matrixToSimulateGame[localMove[6] - '0'][localMove[7] - '0'] = matrixToSimulateGame[localMove[4] - '0'][localMove[5] - '0'];
                    matrixToSimulateGame[localMove[4] - '0'][localMove[5] - '0'] = '.';
                    // Serial.println("Matrix after EAT: ");
                    //  printMatrizGenerica(matrixToSimulateGame, 10, 10);
                }

                // simulating the move
                fullMoves = fullMoves + localMove;
                index_mov++;
            }
        }
        else
        {
            indexA++;
        }
    }
    /*     printf("  File: %d\nMoves in File: %s\nMovements Decoded: %s\n",
               gameToPlay, games[gameToPlay],
               fullMoves.c_str()); */

    // if (gameToPlay < 0 || gameToPlay >= sizeof(games) / sizeof(games[0])) {
    //     Serial.printf("Invalid gameToPlay index: %d\n", gameToPlay);
    // }
    // Serial.printf("File: %d\n", gameToPlay);

    // const char* movesInFile = games[gameToPlay];
    // if (movesInFile == nullptr) {
    //     Serial.printf("Moves in File is null for gameToPlay index: %d\n", gameToPlay);
    // }
    // Serial.printf("Moves in File: %s\n", movesInFile);
    // Serial.printf("Movements Decoded: %s\n", fullMoves.c_str());

    // Serial.println("------------------------------------------------------Reading from file FINISHES");
    return fullMoves;
}

String decodeChessMove(char movChess[7], int cuentaMovimientos)
{
    int int_accion = -1;
    String strChessMove = "";

    if (cuentaMovimientos % 2 == 0)
    {
        strChessMove = "W";
    }
    else
    {
        strChessMove = "B";
    }

    char v_accion[5] = {'-', ' ', 'x', '+', 'O'};
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            if (movChess[i] == v_accion[j])
            {
                if (v_accion[j] == 'x')
                {
                    int_accion = 0;
                }
                if (v_accion[j] == 'O')
                {
                    int_accion++;
                }
            }
        }
    }

    strChessMove += int_accion == -1 ? 0 : int_accion == 0 ? 1
                                                           : int_accion + 1;

    if (int_accion < 1)
    {

        char mask[12] = {'-', ' ', 'x', '+', 'O', 'K', 'Q', 'R', 'B', 'N', 'v'};
        int posIndex[4];
        int k = 0;
        for (int i = 0; i < 7; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                if (movChess[i] == mask[j])
                {
                    break; // If there is a match, exit the inner loop
                }
                if (j == 11)
                {
                    // If we reached the end of the inner loop without a match,
                    // store the position and increment k
                    posIndex[k] = i;
                    k++;
                }
            }
        }
        strChessMove += movChess[posIndex[0]];
        strChessMove += movChess[posIndex[1]];
        strChessMove += movChess[posIndex[2]];
        strChessMove += movChess[posIndex[3]];
        return strChessMove;
    }
    else
    {
        strChessMove += "    ";
        return strChessMove;
    }
}
//============================================================GAME DECODING=====================================================

//==============================================================SENSING======================================================
void sensorsDir()
{
    dirMux[0][0] = 464; // sensor farthest from middle
    dirMux[1][0] = 466;
    dirMux[2][0] = 467;
    dirMux[3][0] = 465;
    dirMux[4][0] = 462;
    dirMux[5][0] = 461;
    dirMux[6][0] = 460;
    dirMux[7][0] = 463;
    dirMux[8][0] = 440;
    dirMux[9][0] = 420; // sensor closest to middle
                        // barra2
    dirMux[0][1] = 414; // sensor farthest from middle
    dirMux[1][1] = 416;
    dirMux[2][1] = 417;
    dirMux[3][1] = 415;
    dirMux[4][1] = 412;
    dirMux[5][1] = 411;
    dirMux[6][1] = 410;
    dirMux[7][1] = 413;
    dirMux[8][1] = 470;
    dirMux[9][1] = 450;

    dirMux[0][2] = 324; // sensor farthest from middle
    dirMux[1][2] = 326;
    dirMux[2][2] = 327;
    dirMux[3][2] = 325;
    dirMux[4][2] = 322;
    dirMux[5][2] = 321;
    dirMux[6][2] = 320;
    dirMux[7][2] = 323;
    dirMux[8][2] = 430;
    dirMux[9][2] = 400;

    dirMux[0][3] = 344; // sensor farthest from middle
    dirMux[1][3] = 346;
    dirMux[2][3] = 347;
    dirMux[3][3] = 345;
    dirMux[4][3] = 342;
    dirMux[5][3] = 341;
    dirMux[6][3] = 340;
    dirMux[7][3] = 343;
    dirMux[8][3] = 360;
    dirMux[9][3] = 310;

    dirMux[0][4] = 334; // sensor farthest from middle
    dirMux[1][4] = 336;
    dirMux[2][4] = 337;
    dirMux[3][4] = 335;
    dirMux[4][4] = 332;
    dirMux[5][4] = 331;
    dirMux[6][4] = 330;
    dirMux[7][4] = 333;
    dirMux[8][4] = 300;
    dirMux[9][4] = 350;

    dirMux[0][5] = 214; // sensor farthest from middle
    dirMux[1][5] = 216;
    dirMux[2][5] = 217;
    dirMux[3][5] = 215;
    dirMux[4][5] = 212;
    dirMux[5][5] = 211;
    dirMux[6][5] = 210;
    dirMux[7][5] = 213;
    dirMux[8][5] = 230;
    dirMux[9][5] = 220;

    dirMux[0][6] = 204; // sensor farthest from middle
    dirMux[1][6] = 206;
    dirMux[2][6] = 207;
    dirMux[3][6] = 205;
    dirMux[4][6] = 202;
    dirMux[5][6] = 201;
    dirMux[6][6] = 200;
    dirMux[7][6] = 203;
    dirMux[8][6] = 240;
    dirMux[9][6] = 250;

    dirMux[0][7] = 154; // sensor farthest from middle
    dirMux[1][7] = 156;
    dirMux[2][7] = 157;
    dirMux[3][7] = 155;
    dirMux[4][7] = 152;
    dirMux[5][7] = 151;
    dirMux[6][7] = 150;
    dirMux[7][7] = 153;
    dirMux[8][7] = 120;
    dirMux[9][7] = 260;

    dirMux[0][8] = 114; // sensor farthest from middle
    dirMux[1][8] = 116;
    dirMux[2][8] = 117;
    dirMux[3][8] = 115;
    dirMux[4][8] = 112;
    dirMux[5][8] = 111;
    dirMux[6][8] = 110;
    dirMux[7][8] = 113;
    dirMux[8][8] = 130;
    dirMux[9][8] = 170;

    dirMux[0][9] = 164; // sensor farthest from middleiiiiiiiiiiiiiiiiiiiiiii
    dirMux[1][9] = 166;
    dirMux[2][9] = 167;
    dirMux[3][9] = 165;
    dirMux[4][9] = 162;
    dirMux[5][9] = 161;
    dirMux[6][9] = 160;
    dirMux[7][9] = 163;
    dirMux[8][9] = 100;
    dirMux[9][9] = 140;
}

void updateSensors()
{
    bool temp[10][10];
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            int muxOut = dirMux[i][j] / 100;       // Get the hundreds digit
            int mux16 = (dirMux[i][j] % 100) / 10; // Get the tens digit
            int mux8 = dirMux[i][j] % 10;          // Get the units digit
            digitalWrite(mux16_0, mux16 & 0b00000001);
            digitalWrite(mux16_1, (mux16 >> 1) & 0b00000001);
            digitalWrite(mux16_2, (mux16 >> 2) & 0b00000001);
            digitalWrite(mux8_0, mux8 & 0b00000001);
            digitalWrite(mux8_1, (mux8 >> 1) & 0b00000001);
            digitalWrite(mux8_2, (mux8 >> 2) & 0b00000001);
            delayMicroseconds(1); // Reduced delay

            temp[i][j] = digitalRead(muxesOut[muxOut]);
        }
    }

    switch (BleChess.getBoardRotation())
    {
    case 0:
        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                sensorUpdate[i][j] = temp[i][j];
            }
        }
        break;
    case 1: // 90
        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                sensorUpdate[10 - j - 1][i] = temp[i][j];
            }
        }
        break;
    case -1: // 180
        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                sensorUpdate[10 - i - 1][10 - j - 1] = temp[i][j];
            }
        }
        break;
    case 2: // 270
        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                sensorUpdate[j][10 - i - 1] = temp[i][j];
            }
        }
        break;
    default:
        for (int i = 0; i < 10; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                sensorUpdate[i][j] = true;
            }
        }
        break;
    }

    sensorUpdate[0][0] = true;
    // sensorUpdate[0][9] = false; THIS IS THE CALIBRATION ONE
    sensorUpdate[9][0] = true;
    sensorUpdate[9][9] = true;
}

void readRawSensors(bool sensorsValue[10][10])
{
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            sensorsValue[i][j] = sensorUpdate[i][j];
        }
    }
}

void detectChessBoard(bool sensVal[10][10])
{
    // mode  = false; normal Mode
    // mode = true; antiBonuncing Mode
    bool sensValAux[10][10];
    bool flag = true;
    int intentos = 0;
    int intentosObjetivo = 3;

    do
    {
        intentos++;
        flag = true;

        readRawSensors(sensVal);

        int tiempoEntreLecturas = (5 * intentos);
        delay(tiempoEntreLecturas);

        readRawSensors(sensValAux);
        for (int i = 1; i < 10; i++)
        {
            for (int j = 0; j < 10; j++)
            {
                if (sensVal[i][j] != sensValAux[i][j])
                {
                    flag = false;
                    Serial.println("Error en la lectura de sensores");
                }
            }
        }

    } while (!flag && intentos <= intentosObjetivo);
}

void desfaseSensado()
{
    // Calculate current position
    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * -(driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / -(driverMicroSteps * 5)) + currentYposition;

    // Calculate the vector from the current position toward the origin (0, 0)
    float vectorX = -currentXposition; // negative to go in the opposite direction toward the origin
    float vectorY = -currentYposition; // negative to go in the opposite direction toward the origin

    // Calculate the magnitude of the current vector from the current position to the origin
    float magnitude = sqrt(vectorX * vectorX + vectorY * vectorY);

    // Calculate the coordinates of the point 50 mm in the opposite direction from the current position
    float xprima = currentXposition + (10 * vectorX / magnitude);
    float yprima = currentYposition + (10 * vectorY / magnitude);

    // Move the system to coordinates xprima, yprima
    rawMovement(xprima, yprima, -4, currentXposition, currentYposition);
}
//===============================================================SENSING=======================================================

//==========================================================MATRIX OPERATIONS=====================================================
void printMatrizGenerica(char matriz[filas][columnas], int filas, int columnas)
{

    char vectNumeros[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    char matrizAux[filas + 1][columnas + 1];

    // Fill matrizAux with the original matrix shifted by one
    for (int j = 1; j < filas + 1; j++)
    {
        for (int i = 1; i < columnas + 1; i++)
        {
            matrizAux[i][j] = matriz[i - 1][j - 1];
        }
    }

    // Add numbers to the top row
    for (int i = 1; i < columnas + 1; i++)
    {
        matrizAux[i][0] = vectNumeros[i - 1];
    }

    // Add numbers to the left column
    for (int j = 1; j < filas + 1; j++)
    {
        matrizAux[0][j] = vectNumeros[j - 1];
    }

    // Space at the top-left corner
    matrizAux[0][0] = ' ';

    // Imprimir matrizAux
    for (int j = 0; j < filas + 1; j++)
    {
        for (int i = 0; i < columnas + 1; i++)
        {
            Serial.print(matrizAux[i][j]);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void initMatrizPlus(char matrixToInit[10][10])
{
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            matrixToInit[i][j] = '.';
        }
    }

    char vectChessW[8] = {'R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R'};
    char vectChessB[8] = {'r', 'n', 'b', 'q', 'k', 'b', 'n', 'r'};

    for (int i = 1; i < 9; i++)
    {
        matrixToInit[i][1] = vectChessB[i - 1];
        matrixToInit[i][2] = 'p';
        matrixToInit[i][8] = vectChessW[i - 1];
        matrixToInit[i][7] = 'P';
    }
}
//==========================================================MATRIX OPERATIONS=====================================================

//==========================================================MECHANICAL CALIBRATION=====================================================
void configAndCurrentPosManager(int setSpeedbyUser, float &driverMicroSteps)
{
    //===============================================DRIVERS CONFIGURATION===============================================
    stepper1.setCurrentPosition(stepper1.currentPosition());
    stepper2.setCurrentPosition(stepper2.currentPosition());
    if (setSpeedbyUser != lastspeedbyUser || driver.microsteps() != driverMicroSteps)
    {
        // digitalWrite(MOTOR_0_DIR_PIN, HIGH);
        // digitalWrite(MOTOR_1_DIR_PIN, HIGH);
        if (setSpeedbyUser <= 200)
        {
            int stepsMultiplier = (driverMicroSteps == 16) ? 8 : (driverMicroSteps == 8) ? 16
                                                                                         : 1;
            stepper1.setCurrentPosition(stepper1.currentPosition() * stepsMultiplier);
            stepper2.setCurrentPosition(stepper2.currentPosition() * stepsMultiplier);
            //  Serial.println("Microsteps 128");
            driverMicroSteps = 128;
        }
        else if (setSpeedbyUser > 200 && setSpeedbyUser < 250)
        {
            int stepsDivider = (driverMicroSteps == 128) ? 8 : (driverMicroSteps == 8) ? 2
                                                                                       : 1;
            stepper1.setCurrentPosition(stepper1.currentPosition() / stepsDivider);
            stepper2.setCurrentPosition(stepper2.currentPosition() / stepsDivider);
            // Serial.println("Microsteps 16");
            driverMicroSteps = 16;
        }
        else
        {
            int stepsDivider = (driverMicroSteps == 128) ? 16 : (driverMicroSteps == 16) ? 2
                                                                                         : 1;
            stepper1.setCurrentPosition(stepper1.currentPosition() / stepsDivider);
            stepper2.setCurrentPosition(stepper2.currentPosition() / stepsDivider);
            // Serial.println("Microsteps 8");
            driverMicroSteps = 8;
        }

        driver.en_spreadCycle(setSpeedbyUser > 200);
        driver.pwm_autograd(true);
        driver.pwm_autoscale(true);
        driver.microsteps(driverMicroSteps);

        driver2.en_spreadCycle(setSpeedbyUser > 200);
        driver2.pwm_autograd(true);
        driver2.pwm_autoscale(true);
        driver2.microsteps(driverMicroSteps);
    }
    /*    Serial.println("Target current: " + String(setSpeedbyUser <= 200 ? MIN_RMS_CURRENT : setSpeedbyUser < 250 ? MIN_RMS_CURRENT + 200
                                                                                                                 : MIN_RMS_CURRENT + 400)); */
    driver.rms_current(setSpeedbyUser <= 200 ? MIN_RMS_CURRENT : setSpeedbyUser < 250 ? MIN_RMS_CURRENT + 100
                                                                                      : MIN_RMS_CURRENT + 200);
    driver2.rms_current(setSpeedbyUser <= 200 ? MIN_RMS_CURRENT : setSpeedbyUser < 250 ? MIN_RMS_CURRENT + 100
                                                                                       : MIN_RMS_CURRENT + 200);
    // Serial.println("Driver Current: " + String(driver.rms_current()));
    // Serial.println("Driver Current 2: " + String(driver2.rms_current()));
    lastspeedbyUser = setSpeedbyUser;
    //===============================================DRIVERS CONFIGURATION===============================================
}

bool testDrivers()
{
    int testOk = 0;
    //-------------------------------MOTOR 1 TEST-------------------------------------------
    uint8_t result_1 = driver.test_connection();
    if (result_1 == 0)
    {
        testOk++;
    }
    driver.rms_current(400);
    driver.microsteps(8);
    if ((driver.rms_current() <= 450 || driver.rms_current() >= 350) && driver.microsteps() == 8)
    {
        testOk++;
    }
    driver.rms_current(1200);
    driver.microsteps(128);
    if ((driver.rms_current() <= 1250 || driver.rms_current() >= 1150) && driver.microsteps() == 128)
    {
        testOk++;
    }
    //-------------------------------MOTOR 2 TEST-------------------------------------------
    uint8_t result_2 = driver2.test_connection();
    if (result_2 == 0)
    {
        testOk++;
    }
    driver2.rms_current(400);
    driver2.microsteps(8);
    if ((driver2.rms_current() <= 450 || driver2.rms_current() >= 350) && driver2.microsteps() == 8)
    {
        testOk++;
    }

    driver2.rms_current(1200);
    driver2.microsteps(128);
    if ((driver2.rms_current() <= 1250 || driver2.rms_current() >= 1150) && driver2.microsteps() == 128)
    {
        testOk++;
    }
    //-------------------------------RESULT-------------------------------------------
    if (testOk == 6)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void rawMovement(float filaEnd, float colEnd, int magnet, float &lastRawpositionX, float &lastRawpositionY)
{
    // electromagnets x, y
    // matrizBin filaend colend

    float coordXEnd = 0;
    float coordYEnd = 0;
    bool sensingFlag = false;

    float electroOffsetX = 0;
    float electroOffsetY = 0;
    float numberofSteps[2] = {0, 0};
    int setSpeedTesting = 300;
    double timeToPosition = 0;
    bool sensorsRM[10][10];

    int speedperStep[2] = {0, 0};

    if (magnet == -2)
    {
        setSpeedTesting = 50;

        coordXEnd = filaEnd;
        coordYEnd = colEnd;
    }
    else if (magnet == -3)
    {
        setSpeedTesting = 300;

        coordXEnd = filaEnd;
        coordYEnd = colEnd;
    }
    else if (magnet == -4)
    {
        setSpeedTesting = 200;
        coordXEnd = filaEnd;
        coordYEnd = colEnd;
    }

    else
    {
        if (magnet == -1)
        {
            magnet = 1;
            sensingFlag = true;
            setSpeedTesting = 15;
        }
        coordXEnd = (50 * filaEnd) - 225;
        coordYEnd = (-50 * colEnd) + 225;
        calcularOffsets(magnet, electroOffsetX, electroOffsetY);
        coordXEnd = coordXEnd + electroOffsetX;
        coordYEnd = coordYEnd + electroOffsetY;
    }

    // -------------------------------------Inverse Kinematics------------------------------------------
    configAndCurrentPosManager(setSpeedTesting, driverMicroSteps);

#ifdef MODELFINAL
    numberofSteps[0] = round(-(driverMicroSteps * 5) * (coordXEnd - coordYEnd));
    numberofSteps[1] = round(-(driverMicroSteps * 5) * (-coordXEnd - coordYEnd));
#endif
#ifdef MODELEFRA
    numberofSteps[0] = round((driverMicroSteps * 5) * (coordXEnd - coordYEnd));
    numberofSteps[1] = round((driverMicroSteps * 5) * (-coordXEnd - coordYEnd));
#endif

    // -------------------------------------Inverse Kinematics------------------------------------------

    // --------------------------------------------Speed------------------------------------------

#ifdef MODELFINAL
    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * -(driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / -(driverMicroSteps * 5)) + currentYposition;
#endif
#ifdef MODELEFRA
    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * (driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / (driverMicroSteps * 5)) + currentYposition;
#endif

    double deltaX = coordXEnd - currentXposition;
    double deltaY = coordYEnd - currentYposition;

    double distance = sqrt(deltaX * deltaX + deltaY * deltaY);

    timeToPosition = distance / setSpeedTesting; // The integer part of the division is the time to reach the position

    speedperStep[0] = ((numberofSteps[0] - stepper1.currentPosition()) / timeToPosition); // assign speed to the speed matrix
    speedperStep[1] = ((numberofSteps[1] - stepper2.currentPosition()) / timeToPosition); // assign speed to the speed matrix
    // --------------------------------------------Speed------------------------------------------

    //  --------------------------------------------------Pure Motor Movement-------------------------------------------------------------------
    stepper1.setMaxSpeed(10000000); // Prevent interference with speed calculations
    stepper2.setMaxSpeed(10000000); // Prevent interference with speed calculations
    float stepsM1 = numberofSteps[0] - stepper1.currentPosition();
    float stepsM2 = numberofSteps[1] - stepper2.currentPosition();

    float initialPosition1 = stepper1.currentPosition();
    float initialPosition2 = stepper2.currentPosition();

    float deltaXSteps = abs(stepsM1);
    float deltaYSteps = abs(stepsM2);

    double stepIsDue = deltaXSteps > deltaYSteps ? static_cast<double>(deltaYSteps) / deltaXSteps : static_cast<double>(deltaXSteps) / deltaYSteps;
    double stepSum = 0.0;
    float dirDeGiro1 = stepsM1 > 0 ? 1 : -1;
    float dirDeGiro2 = stepsM2 > 0 ? 1 : -1;

    /*     int muxOut = dirMux[int(filaEnd)][int(colEnd)] / 100;       // Get the hundreds digit
        int mux16 = (dirMux[int(filaEnd)][int(colEnd)] % 100) / 10; // Get the tens digit
        int mux8 = dirMux[int(filaEnd)][int(colEnd)] % 10;          // Get the units digit
        digitalWrite(mux16_0, mux16 & 0b00000001);
        digitalWrite(mux16_1, (mux16 >> 1) & 0b00000001);
        digitalWrite(mux16_2, (mux16 >> 2) & 0b00000001);
        digitalWrite(mux8_0, mux8 & 0b00000001);
        digitalWrite(mux8_1, (mux8 >> 1) & 0b00000001);
        digitalWrite(mux8_2, (mux8 >> 2) & 0b00000001); */
    bool sensor = true;

    if (deltaXSteps > deltaYSteps)
    {
        stepper1.setAcceleration(speedperStep[0] * 2);
        stepper1.setMaxSpeed(speedperStep[0] * 1);
        stepper1.moveTo(numberofSteps[0]);

        while ((abs(stepper1.currentPosition() - numberofSteps[0]) > 0) && sensor)
        {
            if (sensingFlag == true)
            {
                readRawSensors(sensorsRM);
                sensor = sensorsRM[int(filaEnd)][int(colEnd)];
            }

            stepper1.run();

            if (abs(stepper1.currentPosition() - initialPosition1) >= 1 && sensor)
            {
                stepSum += stepIsDue;
                initialPosition1 = stepper1.currentPosition();
                while (stepSum >= 1 && sensor)
                {
                    if (sensingFlag == true)
                    {
                        readRawSensors(sensorsRM);
                        sensor = sensorsRM[int(filaEnd)][int(colEnd)];
                    }

                    stepper2.setSpeed(1000000.0 * dirDeGiro2);
                    stepper2.runSpeed();
                    stepSum -= 1;
                }
            }
        }
    }
    else
    {
        stepper2.setAcceleration(speedperStep[1] * 2);
        stepper2.setMaxSpeed(speedperStep[1] * 1);
        stepper2.moveTo(numberofSteps[1]);

        while ((abs(stepper2.currentPosition() - numberofSteps[1]) > 0) && sensor)
        {
            if (sensingFlag == true)
            {
                readRawSensors(sensorsRM);
                sensor = sensorsRM[int(filaEnd)][int(colEnd)];
            }

            stepper2.run();

            if (abs(stepper2.currentPosition() - initialPosition2) >= 1 && sensor)
            {
                stepSum += stepIsDue;
                initialPosition2 = stepper2.currentPosition();
                while (stepSum >= 1 && sensor)
                {
                    if (sensingFlag == true)
                    {
                        readRawSensors(sensorsRM);
                        sensor = sensorsRM[int(filaEnd)][int(colEnd)];
                    }
                    stepper1.setSpeed(1000000.0 * dirDeGiro1);
                    stepper1.runSpeed();
                    stepSum -= 1;
                }
            }
        }
    }

    //  --------------------------------------------------Pure Motor Movement-------------------------------------------------------------------
    lastRawpositionY = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * -(driverMicroSteps * 5))); // current position without offset compensation
    lastRawpositionX = (stepper1.currentPosition() / -(driverMicroSteps * 5)) + lastRawpositionY;

    /*     Serial.print("Last Position X: ");
        Serial.print(lastRawpositionX);
        Serial.print(" Y: ");
        Serial.println(lastRawpositionY); */

    lastRawpositionY = lastRawpositionY - coordYEnd;
    lastRawpositionX = lastRawpositionX - coordXEnd;
    /*     if (magnet != -2)
        {
            Serial.print("posX: ");
            Serial.print(lastRawpositionX);
            Serial.print("  posY: ");
            Serial.println(lastRawpositionY);
        } */
    driver.rms_current(HOLD_CURRENT);
    driver2.rms_current(HOLD_CURRENT);
}

void rawMovementStallGuard(float coordXEnd, float coordYEnd, float setSpeedTesting, int activeMotors, int &threshold, int &thresholdD2)
{
    stepper1.setCurrentPosition(stepper1.currentPosition());
    stepper2.setCurrentPosition(stepper2.currentPosition());
    int speedperStep[2] = {0, 0};
    float numberofSteps[2] = {0, 0};

    if (activeMotors == 1)
    {
        driver2.toff(0);
        driver.toff(1);
    }
    else if (activeMotors == 2)
    {
        driver.toff(0);
        driver2.toff(1);
    }
    else
    {
        driver.toff(1);
        driver2.toff(1);
    }

    // -------------------------------------Inverse Kinematics------------------------------------------
    numberofSteps[0] = round(-(driverMicroSteps * 5) * (coordXEnd - coordYEnd));
    numberofSteps[1] = round(-(driverMicroSteps * 5) * (-coordXEnd - coordYEnd));
    // -------------------------------------Inverse Kinematics------------------------------------------

    //  --------------------------------------------Speed------------------------------------------
    float currentYposition = ((stepper1.currentPosition() + stepper2.currentPosition()) / (-2 * -(driverMicroSteps * 5))); // current position without offset compensation
    float currentXposition = (stepper1.currentPosition() / -(driverMicroSteps * 5)) + currentYposition;

    double deltaX = coordXEnd - currentXposition;
    double deltaY = coordYEnd - currentYposition;

    double distance = sqrt(deltaX * deltaX + deltaY * deltaY);

    double timeToPosition = distance / setSpeedTesting; // The integer part of the division is the time to reach the position

    speedperStep[0] = ((numberofSteps[0] - stepper1.currentPosition()) / timeToPosition); // assign speed to the speed matrix
    speedperStep[1] = ((numberofSteps[1] - stepper2.currentPosition()) / timeToPosition); // assign speed to the speed matrix
    //  --------------------------------------------------Pure Motor Movement-------------------------------------------------------------------
    int stepsarstart = 0;
    int stepsarstart2 = 0;

    stepper1.setAcceleration(speedperStep[0] * 1);
    stepper1.setSpeed(speedperStep[0] * 1);
    stepper1.moveTo(numberofSteps[0]);

    stepper2.setAcceleration(speedperStep[1] * 1);
    stepper2.setSpeed(speedperStep[1] * 1);
    stepper2.moveTo(numberofSteps[1]);

    int stall_data_prev_m2 = driver2.SG_RESULT();
    int stall_data_prev_m1 = driver.SG_RESULT();

    const int maxValueCount = 100;         // Maximum number of different values expected
    int values1[maxValueCount] = {0};      // Array to store changeInCalib1 values
    int values2[maxValueCount] = {0};      // Array to store changeInCalib2 values
    int frequencies1[maxValueCount] = {0}; // Array to store the frequency of each changeInCalib1 value
    int frequencies2[maxValueCount] = {0}; // Array to store the frequency of each changeInCalib2 value
    int valueCount1 = 0;                   // Number of distinct values found for changeInCalib1
    int valueCount2 = 0;                   // Number of distinct values found for changeInCalib2

    int mostFrequentValue1 = 0; // The most frequent value for changeInCalib1
    int mostFrequentValue2 = 0; // The most frequent value for changeInCalib2
    unsigned long timeBeforeMotors = millis();
    bool flagOnce = false;
    do
    {
        int distToReadStall = 320; // 240;
        int current_sg_result2 = driver2.SG_RESULT();
        int current_sg_result1 = driver.SG_RESULT();
        int changeInCalib2 = abs(stall_data_prev_m2 - current_sg_result2);
        int changeInCalib1 = abs(stall_data_prev_m1 - current_sg_result1);
        if (timeBeforeMotors + 50 < millis() && !flagOnce) //
        {
            if (timer1H == NULL)
            {
                timer1H = timerBegin(3, 2, true);
                timerAttachInterrupt(timer1H, &moveRawStallGuard, true);
                timerAlarmWrite(timer1H, 400, true);
            }
            timerAlarmEnable(timer1H);
            stepsarstart = stepper1.currentPosition();
            stepsarstart2 = stepper2.currentPosition();
            flagOnce = true;
        }

        // Search for the value in the array for changeInCalib1
        bool found1 = false;
        for (int i = 0; i < valueCount1; i++)
        {
            if (values1[i] == changeInCalib1)
            {
                frequencies1[i]++;
                found1 = true;
                break;
            }
        }

        // If the value is not in the array, add it for changeInCalib1
        if (!found1 && valueCount1 < maxValueCount)
        {
            values1[valueCount1] = changeInCalib1;
            frequencies1[valueCount1] = 1;
            valueCount1++;
        }

        // Search for the value in the array for changeInCalib2
        bool found2 = false;
        for (int i = 0; i < valueCount2; i++)
        {
            if (values2[i] == changeInCalib2)
            {
                frequencies2[i]++;
                found2 = true;
                break;
            }
        }

        // If the value is not in the array, add it for changeInCalib2
        if (!found2 && valueCount2 < maxValueCount)
        {
            values2[valueCount2] = changeInCalib2;
            frequencies2[valueCount2] = 1;
            valueCount2++;
        }

        bool distFlag = false;
        if (activeMotors == 1)
        {
            if (abs(stepper1.currentPosition() - stepsarstart) > distToReadStall)
            {
                distFlag = true;
            }
        }
        else if (activeMotors == 2)
        {
            if (abs(stepper2.currentPosition() - stepsarstart2) > distToReadStall)
            {
                distFlag = true;
            }
        }
        else
        {
            if ((abs(stepper1.currentPosition() - stepsarstart) > distToReadStall) && (abs(stepper2.currentPosition() - stepsarstart2) > distToReadStall))
            {
                distFlag = true;
            }
        }
        // Serial.printf("dif M1: %d dif M2: %d difposM1:%d difposM2: %d \n", changeInCalib1, changeInCalib2, abs(stepper1.currentPosition() - stepsarstart), abs(stepper2.currentPosition() - stepsarstart2));
        // Serial.printf("dif M1: %d dif M2: %d  \n", changeInCalib1, changeInCalib2);
        if (distFlag && (changeInCalib1 > threshold || changeInCalib2 > thresholdD2))
        {
            // Serial.println("-----------------------------Stall Detected");
            break;
        }
        stall_data_prev_m2 = current_sg_result2;
        stall_data_prev_m1 = current_sg_result1;
    } while (stepper2.distanceToGo() != 0 || stepper1.distanceToGo() != 0);

    // Disable and stop the timer
    driver2.toff(0);
    driver.toff(0);

    if (timer1H != NULL)
    {
        timerAlarmDisable(timer1H);
        timerDetachInterrupt(timer1H);
        timerEnd(timer1H);
        timer1H = NULL; // Ensure the pointer is NULL after freeing the timer
        // Serial.println("==============================================Timer disabled");
    }

    for (int i = 0; i < valueCount1; i++)
    {
        // Serial.printf("Value1: %d, Frequency1: %d\n", values1[i], frequencies1[i]);
        // verify the highest value; it must be large and have a frequency greater than 5
        if (frequencies1[i] > 5 && values1[i] > mostFrequentValue1)
        {
            mostFrequentValue1 = values1[i];
        }
    }
    // Serial.println("-------------");
    for (int i = 0; i < valueCount2; i++)
    {
        // Serial.printf("Value2: %d, Frequency2: %d\n", values2[i], frequencies2[i]);
        //  verify the highest value; it must be large and have a frequency greater than 5
        if (frequencies2[i] > 5 && values2[i] > mostFrequentValue2)
        {
            mostFrequentValue2 = values2[i];
        }
    }

    threshold = mostFrequentValue1;
    thresholdD2 = mostFrequentValue2;
    // Serial.printf("High and Frequent Value1: %d High and Frequent Value2: %d\n", mostFrequentValue1, mostFrequentValue2);
}

void IRAM_ATTR moveRawStallGuard()
{
    stepper1.runSpeedToPosition();
    stepper2.runSpeedToPosition();
}

void configDrivers()
{
    int tbllocal = 1;
    int hstrtlocal = 4;
    int hstrtemdlocal = 2;

    driver.begin();

    driver.tbl(tbllocal);
    driver.I_scale_analog(false);  // disabled to use the ecternal current sense resistor
    driver.internal_Rsense(false); // Same as above
    driver.hstrt(hstrtlocal);
    driver.hend(hstrtemdlocal);

    driver.irun(31);
    driver.ihold(16);
    driver.iholddelay(4);
    driver.TPOWERDOWN(2);

    driver.intpol(true);
    driver.pwm_autograd(true);
    driver.pwm_autoscale(true);

    driver2.begin();

    driver2.tbl(tbllocal);          // For most applications 1 or 2 is fine; for high-load applications use 2 or 3
    driver2.I_scale_analog(false);  // For most applications it is better to disable this
    driver2.internal_Rsense(false); // For most applications it is better to disable this
    driver2.hstrt(hstrtlocal);      // datasheet page 50; low values reduce precision and high values produce more noise.
    driver2.hend(hstrtemdlocal);    // datasheet page 50; low values reduce precision and high values produce more noise.

    driver2.irun(31);
    driver2.ihold(16);
    driver2.iholddelay(4);
    driver2.TPOWERDOWN(2);

    driver2.intpol(true);
    driver2.pwm_autograd(true);
    driver2.pwm_autoscale(true);

    digitalWrite(ENABLE_PIN, HIGH);
    driver.rms_current(HOLD_CURRENT);  // Temperature and current consumption protection
    driver2.rms_current(HOLD_CURRENT); // Temperature and current consumption protection
    delay(150);
}

void mechanicalCalibration(int calibrationMode)
{
    Serial.println("HOMING STARTED " + String(calibrationMode));
    BleChess.setState("Homing");

    preferences.begin("myApp", true);
    int stallOld = preferences.getInt("stallOld", 40);
    int stallOlder = preferences.getInt("stallOlder", 40);
    int stallOldD2 = preferences.getInt("stallOldD2", 40);
    int stallOlderD2 = preferences.getInt("stallOlderD2", 40);
    int enhancedCalib = preferences.getInt("betterCalib", 1);
    preferences.end();

    int stalltoUse = (stallOld > stallOlder) ? stallOld : stallOlder;
    stalltoUse = (stalltoUse <= 0 || stalltoUse > 40) ? 40 : stalltoUse;
    stalltoUse = stalltoUse + 10;
    int stalltoUseD2 = (stallOldD2 > stallOlderD2) ? stallOldD2 : stallOlderD2;
    stalltoUseD2 = (stalltoUseD2 <= 0 || stalltoUseD2 > 40) ? 40 : stalltoUseD2;
    stalltoUseD2 = stalltoUseD2 + 10;
    Serial.printf("Stall Old: %d Stall Older: %d Stall to use: %d StallD2 Old: %d StallD2 Older: %d StallD2 to Use: %d \n", stallOld, stallOlder, stalltoUse, stallOldD2, stallOlderD2, stalltoUseD2);

    float fastSpeed = 250000; // 250000
    float slowSpeed = fastSpeed / 10;
    driverMicroSteps = 32; // Ref 10000, curr 1000//128 -> 10,40 | 64 -> 22,50 | 32 -> 36, 66 | 16 -> 52,70 | 8 -> 74,94;70,80

    int activeMotors = -1;
    bool drv1 = true;
    bool drv2 = true;
    if (calibrationMode == 0)
    {
        Serial.println("===============================================================CALIBRATION MODE 0");
        activeMotors = 2;
    }
    else
    {
        Serial.println("===============================================================CALIBRATION MODE 1");
    }

    if (activeMotors == 1)
    {
        drv1 = false;
        driver.rms_current(CALIB_CURRENT);
        driver.microsteps(driverMicroSteps);
        driver.en_spreadCycle(false); // Enable StealthChop

        driver2.rms_current(0);
        driver2.microsteps(driverMicroSteps);
        driver2.en_spreadCycle(false); // Enable StealthChop
    }
    else if (activeMotors == 2)
    {
        drv2 = false;
        driver.rms_current(0);
        driver.microsteps(driverMicroSteps);
        driver.en_spreadCycle(false); // Enable StealthChop

        driver2.rms_current(CALIB_CURRENT);
        driver2.microsteps(driverMicroSteps);
        driver2.en_spreadCycle(false); // Enable StealthChop
    }
    else
    {
        drv1 = false;
        drv2 = false;
        driver.rms_current(CALIB_CURRENT - 300);
        driver.microsteps(driverMicroSteps);
        driver.en_spreadCycle(false); // Enable StealthChop

        driver2.rms_current(CALIB_CURRENT - 300);
        driver2.microsteps(driverMicroSteps);
        driver2.en_spreadCycle(false); // Enable StealthChop
    }

    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);

    int toleranciaM = driverMicroSteps * 5 * 5; // equivalent to 800 steps, which is 5mm

    int stallCounter = 0;
    int maxValueStall = 0;
    int maxValueStallD2 = 0;
    if (calibrationMode == 0)
    {
        do
        {
            readRawSensors(matrizBinSc);
            int stallVolatile = stalltoUse;
            int stallVolatileD2 = stalltoUseD2;
            rawMovementStallGuard(-500, -500, fastSpeed, activeMotors, stallVolatile, stallVolatileD2);
            maxValueStall = max(maxValueStall, stallVolatile);
            maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            stallCounter++;
            Serial.printf("current Position M1: %d current Position M2: %d toleranciaM: %d\n", stepper1.currentPosition(), stepper2.currentPosition(), toleranciaM);
            if (matrizBinSc[0][9] && enhancedCalib == 1)
            {
                Serial.println("HOMING POR SENSOR 09");
                activateElectromagnetV2(4, 35);
                delay(300);

                int sensor44FirstStatus = matrizBinSc[4][4];
                readRawSensors(matrizBinSc);
                Serial.println("Sensor Status After Stall: " + String(matrizBinSc[0][9]));

                if (!matrizBinSc[0][9]) // The sensor was used and detected; we are at the correct position
                {
                    deactivateAllMagnets();
                    Serial.println("TERMINA HOMING POR SENSOR 09");
                    break;
                }
                else // Either the sensor is not working or too many attempts were made without reaching the position due to insufficient current.
                {
                    if (sensor44FirstStatus == 1 && matrizBinSc[4][4] == 0)
                    {
                        Serial.println("We are probably locked in, trying to unlock");
                        unsigned long timer = millis();
                        BleChess.sendTestModeError("MECHANISM LOCKED, REMOVE LOCK UNDER THE BOARD AND RESTART");
                        while (true)
                        {
                            if (millis() - timer > 5000)
                            {
                                BleChess.sendTestModeError("MECHANISM LOCKED, REMOVE LOCK UNDER THE BOARD AND RESTART");
                                timer = millis();
                            }
                            cute._tone(NOTE_G3, 50, 100);
                            delay(100);
                            cute._tone(NOTE_C0, 50, 100);
                        }
                    }
                    else
                    {
                        Serial.println("NO SE DETECTO EL SENSOR 09, REINTENTANDO");
                        deactivateAllMagnets();
                        if (stallCounter == 3) // TOO MANY ATTEMPS, DEACTIVATE ENHANCED CALIBRATION
                        {
                            BleChess.sendTestModeError("Enhanced Homing went wrong. Send SS of this to Phantom Team and Restart your board please. ");
                            enhancedCalib = 0;
                            preferences.begin("myApp", false);
                            preferences.putInt("betterCalib", enhancedCalib);
                            preferences.end();
                        }
                    }
                }
                deactivateAllMagnets();
            }
            else
            {
                Serial.println("HOMING POR TOLERANCIAS");
                if ((drv1 || abs(stepper1.currentPosition()) <= toleranciaM) && (drv2 || abs(stepper2.currentPosition()) <= toleranciaM))
                {
                    Serial.printf("Stall in Range\n");
                    break;
                }
            }

            stepper1.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);

            if (stallCounter < 4)
            {
                Serial.printf("PREPARANDO OTRO ROUND DE HOMING \n\n\n\n");
                stallVolatile = stalltoUse;
                stallVolatileD2 = stalltoUseD2;
                rawMovementStallGuard(10, 10, slowSpeed, activeMotors, stallVolatile, stallVolatileD2);
                maxValueStall = max(maxValueStall, stallVolatile);
                maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            }
            else
            {
                Serial.printf("TOO MANY ATTEMPS, DEACTIVATE ENHANCED CALIBRATION \n\n\n\n");
                break;
            }

            Serial.println("=========================Stall counter: " + String(stallCounter));

        } while (true);

        preferences.begin("myApp", false);
        preferences.putInt("stallOld", maxValueStall);
        preferences.putInt("stallOlder", stallOld);
        preferences.putInt("stallOldD2", maxValueStallD2);
        preferences.putInt("stallOlderD2", stallOlder);
        preferences.end();
        Serial.printf("Max Value Stall: %d Max Value Stall D2: %d \n", maxValueStall, maxValueStallD2);

        driver.toff(TOFFTEST);
        driver2.toff(TOFFTEST);
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);

        int coordXEnd = -210;
        int coordYEnd = -198.5;
        driverMicroSteps = 128;
        int stepsM1 = round(-(driverMicroSteps * 5) * (coordXEnd - coordYEnd));
        int stepsM2 = round(-(driverMicroSteps * 5) * (-coordXEnd - coordYEnd));
        stepper1.setCurrentPosition(stepsM1);
        stepper2.setCurrentPosition(stepsM2);
    }
    else
    {
        do
        {
            bool sensorsX9 = true;
            readRawSensors(matrizBinSc);
            int stallVolatile = stalltoUse;
            int stallVolatileD2 = stalltoUseD2;
            rawMovementStallGuard(500, 0, fastSpeed, activeMotors, stallVolatile, stallVolatileD2);
            maxValueStall = max(maxValueStall, stallVolatile);
            maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            stallCounter++;
            // Serial.printf("current Position M1: %d current Position M2: %d toleranciaM: %d\n", stepper1.currentPosition(), stepper2.currentPosition(), toleranciaM);

            for (int i = 1; i < 9; i++)
            {
                if (!matrizBinSc[9][i])
                {
                    Serial.println("sensor: X9 Y" + String(i) + " is on");
                    sensorsX9 = false;
                }
                else
                {
                    Serial.println("----------------------All sensors X9 turned off");
                }
            }

            if (sensorsX9 && enhancedCalib == 1)
            {
                // sensorsX9 = true;
                Serial.println("HOMING POR SENSOR X9");
                activateElectromagnetV2(3, 35);
                delay(300);

                readRawSensors(matrizBinSc);

                for (int i = 1; i < 9; i++)
                {
                    if (!matrizBinSc[9][i])
                    {
                        Serial.println("sensor: X9 Y" + String(i) + " is on");
                        sensorsX9 = false;
                    }
                    else
                    {
                        Serial.println("----------------------All sensors X9 turned off");
                    }
                }

                if (!sensorsX9) // The sensor was used and detected; we are at the correct position
                {
                    deactivateAllMagnets();
                    Serial.println("TERMINA HOMING POR SENSOR X9");
                    break;
                }
                else // Either the sensor is not working or too many attempts were made without reaching the position due to insufficient current.
                {
                    Serial.println("NO SE DETECTO EL SENSOR X9, REINTENTANDO");
                    enhancedCalib = 0;
                    deactivateAllMagnets();
                }
                deactivateAllMagnets();
            }
            else
            {
                Serial.println("HOMING POR TOLERANCIAS");
                if ((drv1 || abs(stepper1.currentPosition()) <= toleranciaM) && (drv2 || abs(stepper2.currentPosition()) <= toleranciaM))
                {
                    Serial.printf("Stall in Range\n");
                    break;
                }
            }

            stepper1.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);

            if (stallCounter < 4)
            {
                // Serial.printf("PREPARING ANOTHER HOMING ROUND \n\n\n\n");
                stallVolatile = stalltoUse;
                stallVolatileD2 = stalltoUseD2;
                rawMovementStallGuard(-10, 0, slowSpeed, activeMotors, stallVolatile, stallVolatileD2);
                maxValueStall = max(maxValueStall, stallVolatile);
                maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            }
            else
            {
                // Serial.printf("TOO MANY ATTEMPS, DEACTIVATE ENHANCED CALIBRATION \n\n\n\n");
                break;
            }

            Serial.println("=========================Stall counter: " + String(stallCounter));
        } while (true);

        enhancedCalib = 1;
        driver.toff(TOFFTEST);
        driver2.toff(TOFFTEST);
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);
        float posX, posY;
        rawMovement(-208.5 - 25, 0, -2, posX, posY);

        driverMicroSteps = 32;
        driver.rms_current(CALIB_CURRENT - 300);
        driver.microsteps(driverMicroSteps);
        driver.en_spreadCycle(false); // Enable StealthChop
        driver2.rms_current(CALIB_CURRENT - 300);
        driver2.microsteps(driverMicroSteps);
        driver2.en_spreadCycle(false); // Enable StealthChop
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);

        stallCounter = 0;

        do
        {
            readRawSensors(matrizBinSc);
            int stallVolatile = stalltoUse;
            int stallVolatileD2 = stalltoUseD2;
            rawMovementStallGuard(0, -500, fastSpeed, activeMotors, stallVolatile, stallVolatileD2);
            maxValueStall = max(maxValueStall, stallVolatile);
            maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            stallCounter++;
            // Serial.printf("current Position M1: %d current Position M2: %d toleranciaM: %d\n", stepper1.currentPosition(), stepper2.currentPosition(), toleranciaM);

            if (matrizBinSc[4][9] && enhancedCalib == 1)
            {
                Serial.println("HOMING POR SENSOR 49");
                activateElectromagnetV2(4, 35);
                delay(300);

                readRawSensors(matrizBinSc);
                Serial.println("Sensor Status After Stall: " + String(matrizBinSc[4][9]));

                if (!matrizBinSc[4][9]) // The sensor was used and detected; we are at the correct position
                {
                    deactivateAllMagnets();
                    Serial.println("TERMINA HOMING POR SENSOR 49");
                    break;
                }
                else // Either the sensor is not working or too many attempts were made without reaching the position due to insufficient current.
                {
                    Serial.println("NO SE DETECTO EL SENSOR 49, REINTENTANDO");
                    deactivateAllMagnets();
                    if (stallCounter == 3) // TOO MANY ATTEMPS, DEACTIVATE ENHANCED CALIBRATION
                    {
                        BleChess.sendTestModeError("Enhanced Homing went wrong. Send SS of this to Phantom Team and Restart your board please. ");
                        enhancedCalib = 0;
                        preferences.begin("myApp", false);
                        preferences.putInt("betterCalib", enhancedCalib);
                        preferences.end();
                    }
                }
                deactivateAllMagnets();
            }
            else
            {
                Serial.println("HOMING POR TOLERANCIAS");
                if ((drv1 || abs(stepper1.currentPosition()) <= toleranciaM) && (drv2 || abs(stepper2.currentPosition()) <= toleranciaM))
                {
                    Serial.printf("Stall in Range\n");
                    break;
                }
            }

            stepper1.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);

            if (stallCounter < 4)
            {
                // Serial.printf("PREPARING ANOTHER HOMING ROUND \n\n\n\n");
                stallVolatile = stalltoUse;
                stallVolatileD2 = stalltoUseD2;
                rawMovementStallGuard(0, 10, slowSpeed, activeMotors, stallVolatile, stallVolatileD2);
                maxValueStall = max(maxValueStall, stallVolatile);
                maxValueStallD2 = max(maxValueStallD2, stallVolatileD2);
            }
            else
            {
                // Serial.printf("TOO MANY ATTEMPS, DEACTIVATE ENHANCED CALIBRATION \n\n\n\n");
                break;
            }

            // Serial.println("=========================Stall counter: " + String(stallCounter));
        } while (true);

        preferences.begin("myApp", false);
        preferences.putInt("stallOld", maxValueStall);
        preferences.putInt("stallOlder", stallOld);
        preferences.putInt("stallOldD2", maxValueStallD2);
        preferences.putInt("stallOlderD2", stallOlder);
        preferences.end();
        Serial.printf("Max Value Stall: %d Max Value Stall D2: %d \n", maxValueStall, maxValueStallD2);

        driver.toff(TOFFTEST);
        driver2.toff(TOFFTEST);
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);

        int desfaseenX = -5;
        int desfaseenY = 3;
        int coordXEnd = -25 + (stallCounter * desfaseenX);
        int coordYEnd = -198.5 + desfaseenY;

        driverMicroSteps = 128;
        int stepsM1 = round(-(driverMicroSteps * 5) * (coordXEnd - coordYEnd));
        int stepsM2 = round(-(driverMicroSteps * 5) * (-coordXEnd - coordYEnd));
        stepper1.setCurrentPosition(stepsM1);
        stepper2.setCurrentPosition(stepsM2);
    }

    driver.rms_current(HOLD_CURRENT);
    driver2.rms_current(HOLD_CURRENT);
    BleChess.setState("");
    Serial.println("HOMING FINISHED");

    /*   float posX, posY;
     rawMovement(0, 0, -2, posX, posY);
     while (true)
     {

     }  */
}

//==========================================================MECHANICAL CALIBRATION=====================================================
void sensorsCalibration(int filaEnd, int colEnd, int modo, float &totalX, float &totalY)
{
    // mode 0 -> sensor calibration
    // mode 1 -> sensor calibration with correction
    float posX = 0;
    float posY = 0;
    float anguloPizza = 60;
    float electroOffsetX = 0;
    float electroOffsetY = 0;
    int intentos = 0;

    float coordXEnd = (50 * filaEnd) - 225;
    float coordYEnd = (-50 * colEnd) + 225;
    calcularOffsets(1, electroOffsetX, electroOffsetY);
    coordXEnd = coordXEnd + electroOffsetX;
    coordYEnd = coordYEnd + electroOffsetY;

    activateElectromagnetV2(1, 25);
    delay(250);
    do
    {
        totalX = 0;
        totalY = 0;
        for (int angulo = 0; angulo < 360; angulo += anguloPizza)
        {
            float radianes = angulo * PI / 180.0;
            float x = coordXEnd + 50 * cos(radianes);
            float y = coordYEnd + 50 * sin(radianes);

            rawMovement(x, y, -2, posX, posY);
            rawMovement(filaEnd, colEnd, -1, posX, posY);

            totalX = totalX + posX;
            totalY = totalY + posY;
        }
        totalX = totalX / (360 / anguloPizza);
        totalY = totalY / (360 / anguloPizza);

        Serial.print("error x");
        Serial.print(totalX);
        Serial.print("  error y");
        Serial.println(totalY);

        if (modo == 1)
        {
            intentos++;
            Serial.println("Medida Correctiva: ");
            /*             float correctX = (totalX > 4 || totalX < -4) ? totalX * 1.75 : (totalX > 2 || totalX < -2) ? totalX * 1.5
                                                                                                                   : totalX * 1.25;
                        float correctY = (totalY > 4 || totalY < -4) ? totalY * 1.75 : (totalY > 2 || totalY < -2) ? totalY * 1.5
                                                                                                                   : totalY * 1.25; */

            float correctX = totalX * 1.75;
            float correctY = totalY * 1.75;

            Serial.print("correct x");
            Serial.print(correctX);
            Serial.print("  correct  y");
            Serial.println(correctY);

            rawMovement(correctX, correctY, -2, posX, posY);
            stepper1.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);
        }
        Serial.println("============================================");

    } while (modo == 1 && intentos <= 5 && (totalX <= -0.5 || totalX >= 0.5 || totalY <= -0.5 || totalY >= 0.5));
    deactivateAllMagnets();
}

void setCurrentPosition(float x, float y)
{
    driverMicroSteps = 128;
    int m1 = round(-(driverMicroSteps * 5) * (x - y));
    int m2 = round(-(driverMicroSteps * 5) * (-x - y));

    stepper1.setCurrentPosition(m1);
    stepper2.setCurrentPosition(m2);
}

void soundHandler(int soundMode)
{
    int _sound = BleChess.getSoundLevel();

    // Serial.print("Sound Mode: " + String(soundMode) + " Sound Level: " + _sound + "\n");
    switch (soundMode)
    {
    case 0: // Special sound start product
        cute.play(S_CONNECTION);
        break;
    case 1: // Alert sound
        cute._tone(NOTE_C7, 50, 30);
        cute._tone(NOTE_C7, 50, 30);
        cute._tone(NOTE_C7, 50, 30);
        cute._tone(NOTE_C7, 50, 30);
        cute._tone(NOTE_C7, 50, 30);
        cute._tone(NOTE_C7, 50, 30);
        break;
    case 2: // Alert to change special modes
        cute._tone(NOTE_C7, 50, 30);
        break;
    case 3: // Testing sound
        cute.play(S_FART1);
        cute.play(S_FART2);
        cute.play(S_FART3);
        break;
    case 4: // Incorrect Movement
        if (_sound == 3)
        {
            cute._tone(NOTE_C7, 200, 10);
            delay(100);
            cute._tone(NOTE_C7, 200, 10);
        }
        else if (_sound == 2)
        {
            cute._tone(NOTE_A2, 50, 30);
            delay(50);
            cute._tone(NOTE_A2, 50, 30);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A0, 50, 30);
            delay(50);
            cute._tone(NOTE_A0, 50, 30);
        }
        break;
    case 5: // Correct movement sound
        if (_sound == 3)
        {
            cute._tone(NOTE_C7, 200, 10);
        }
        else if (_sound == 2)
        {
            cute._tone(NOTE_C4, 50, 30);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A0, 50, 30);
        }
        break;
    case 6: // Special sound for unmatch chessboard
        if (_sound == 2)
        {
            cute.play(S_CONNECTION);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A0, 50, 30);
        }
        break;
    case 7: // Special sound for factory Reset
        if (_sound == 2)
        {
            cute.play(S_MODE3);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A0, 50, 30);
        }
        break;
    case 8: // Special sound for no battery
        cute.play(S_DISCONNECTION);
        break;
    case 9: // Special sound for battery
        cute.play(PIRATES);
        break;
    case 10: // check
        if (_sound == 2)
        {
            delay(1000);
            cute._tone(NOTE_Gb3, 1000, 30);
            delay(100);
            cute._tone(NOTE_Gb3, 1000, 30);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A0, 1000, 30);
            delay(50);
            cute._tone(NOTE_A0, 1000, 30);
        }
        break;
    case 11: // checkmate
        if (_sound == 2)
        {
            cute._tone(NOTE_Gb3, 2000, 30);
        }
        else if (_sound == 1)
        {
            cute._tone(NOTE_A1, 2000, 30);
        }
        break;

    default:
        Serial.print("Invalid Sound -----Bug detected 0x01 ");
        break;
    }
}

int buscarPosicionCercanaVacia(int filaActualAux, int colActualAux, char matrixToSearch[10][10])
{
    int filaActual = filaActualAux; // Current row being searched around
    int colActual = colActualAux;   // Current column being searched around

    int rango = 1; // Initialize the search range

    while (true)
    {
        // Explore positions within the current range
        for (int j = -rango; j <= rango; ++j) // explore row
        {
            for (int i = -rango; i <= rango; ++i) // explore column
            {
                int nuevaFila = filaActual + i;
                int nuevaCol = colActual + j;

                // Ensure the new position is within the matrix bounds and not at a corner
                if (nuevaFila >= 0 && nuevaFila < 10 && nuevaCol >= 0 && nuevaCol < 10 && !(nuevaFila == 0 && nuevaCol == 0) && !(nuevaFila == 0 && nuevaCol == 9) && !(nuevaFila == 9 && nuevaCol == 0) && !(nuevaFila == 9 && nuevaCol == 9))
                {
                    // Check if the position is empty
                    if (matrixToSearch[nuevaFila][nuevaCol] == '.' || matrizBinSc[nuevaFila][nuevaCol] == 1)
                    {
                        // Found the nearest empty position
                        return (nuevaFila * 10) + nuevaCol; // Return the nearest position if only one is needed
                    }
                }
            }
        }

        // If the value is not found in the current range, increase the range for the next iteration
        rango++;

        //  You can add a condition to exit the loop if the range becomes too large
        if (rango > 11)
        {
            // Serial.println("When Reordering, not found an empty position.");
            return -1;
        }
    }
}

int buscarPosicionCercanaPieza(int filaActualAux, int colActualAux, char piezaObjetivo, char matrixToSearch[10][10])
{
    int filaActual = filaActualAux; // Current row being searched around
    int colActual = colActualAux;   // Current column being searched around

    int rango = 1; // Initialize the search range

    while (true)
    {
        // Explore positions within the current range
        for (int j = -rango; j <= rango; ++j) // explore row
        {
            for (int i = -rango; i <= rango; ++i) // explore column
            {
                int nuevaFila = filaActual + i;
                int nuevaCol = colActual + j;

                // Ensure the new position is within the matrix bounds and not at a corner
                if (nuevaFila >= 0 && nuevaFila < 10 && nuevaCol >= 0 && nuevaCol < 10 && !(nuevaFila == 0 && nuevaCol == 0) && !(nuevaFila == 0 && nuevaCol == 9) && !(nuevaFila == 9 && nuevaCol == 0) && !(nuevaFila == 9 && nuevaCol == 9))
                {
                    // Check if the position holds the target piece
                    if (matrizObjetivo[nuevaFila][nuevaCol] != 'L' && matrixToSearch[nuevaFila][nuevaCol] == piezaObjetivo && matrizBinSc[nuevaFila][nuevaCol] == 0)
                    {
                        // Found the nearest matching position
                        return (nuevaFila * 10) + nuevaCol; // Return the nearest position if only one is needed
                    }
                }
            }
        }

        // If the value is not found in the current range, increase the range for the next iteration
        rango++;

        //  You can add a condition to exit the loop if the range becomes too large
        if (rango > 11)
        {
            // Serial.println("When Reordering, not found a close piece.");
            return -1;
        }
    }
}
