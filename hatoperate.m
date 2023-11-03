function hat = hatoperate(w_t);
         hat = [0 w_t(1,1) w_t(2,1);-w_t(1,1) 0 w_t(3,1);-w_t(2,1) -w_t(3,1) 0];
end