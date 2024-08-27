/*
 * cli.h
 *
 *  Created on: Sep 21, 2022
 *      Author: matveev
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_

void initCli();
void processCli(char c);
void printCli (const char * str);
int executeCli (int argc, const char * const * argv);
char ** completeCli (int argc, const char * const * argv);
char * promptCli();

#endif /* INC_CLI_H_ */
