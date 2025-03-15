import numpy as np

import torch
import torch.nn as nn
from torch.distributions import Normal
import torch.nn.functional as F 
from modules.common_modules import AutoEncoder, BetaVAE, RnnBarlowTwinsStateHistoryEncoder, RnnStateHistoryEncoder, StateHistoryEncoder, get_activation, mlp_factory, mlp_layernorm_factory
from modules.transformer_modules import StateCausalTransformer
class Config:
    def __init__(self):
        self.n_obs = 45
        self.block_size = 9
        self.n_action = 45+3
        self.n_layer: int = 4
        self.n_head: int = 4
        self.n_embd: int = 32
        self.dropout: float = 0.0
        self.bias: bool = True

class CnnActor(nn.Module):
    def __init__(self,
                 num_prop,
                 num_hist,
                 num_actions,
                 priv_encoder_output_dim,
                 actor_hidden_dims=[256, 256, 256],
                 activation='elu'):
        super(CnnActor,self).__init__()
        self.num_prop = num_prop
        self.num_hist = num_hist
        self.num_actions = num_actions
        self.priv_encoder_output_dim = priv_encoder_output_dim
        self.activation = activation
        self.history_encoder = StateHistoryEncoder(activation, num_prop, num_hist, priv_encoder_output_dim)
        self.actor_layers = mlp_factory(activation,num_prop+priv_encoder_output_dim,num_actions,actor_hidden_dims,last_act=False)
        self.actor = nn.Sequential(*self.actor_layers)
    
    def forward(self,obs,hist):
        latent = self.history_encoder(hist)
        backbone_input = torch.cat([obs,latent], dim=1)
        mean = self.actor(backbone_input)
        return mean
    
class RnnActor(nn.Module):
    def __init__(self,
                 num_prop,
                 encoder_dims,
                 decoder_dims,
                 actor_dims,
                 encoder_output_dim,
                 hidden_dim,
                 num_actions,
                 activation,) -> None:
        super(RnnActor,self).__init__()
        self.rnn_encoder = RnnStateHistoryEncoder(activation_fn=activation,
                                                  input_size=num_prop,
                                                  encoder_dims=encoder_dims,
                                                  hidden_size=hidden_dim,
                                                  output_size=encoder_output_dim)
        self.next_state_decoder =nn.Sequential(*mlp_factory(activation=activation,
                                              input_dims=hidden_dim,
                                              out_dims=num_prop+7,
                                              hidden_dims=decoder_dims))

        self.actor = nn.Sequential(*mlp_factory(activation=activation,
                                 input_dims=hidden_dim + num_prop,
                                 out_dims=num_actions,
                                 hidden_dims=actor_dims))

    def forward(self,obs,obs_hist):
        obs_hist_full = torch.cat([
                obs_hist[:,1:,:],
                obs.unsqueeze(1)
            ], dim=1)
        latents = self.rnn_encoder(obs_hist_full)
        actor_input = torch.cat([latents[:,-1,:],obs],dim=-1)
        mean  = self.actor(actor_input)
        return mean

    def predict_next_state(self,obs_hist):
        # self.rnn_encoder.reset_hidden()
        latents = self.rnn_encoder(obs_hist)
        predicted = self.next_state_decoder(latents[:,-1,:])
        return predicted
    
class RnnBarlowTwinsActor(nn.Module):
    def __init__(self,
                 num_prop,
                 obs_encoder_dims,
                 rnn_encoder_dims,
                 actor_dims,
                 encoder_output_dim,
                 hidden_dim,
                 latent_dim,
                 num_actions,
                 activation) -> None:
        super(RnnBarlowTwinsActor,self).__init__()
        self.rnn_encoder = RnnBarlowTwinsStateHistoryEncoder(activation_fn=activation,
                                                  input_size=num_prop,
                                                  hidden_size=hidden_dim,
                                                  output_size=encoder_output_dim,
                                                  final_output_size=latent_dim,
                                                  encoder_dims=rnn_encoder_dims)

        self.actor = nn.Sequential(*mlp_factory(activation=activation,
                                 input_dims=latent_dim + num_prop,
                                 out_dims=num_actions,
                                 hidden_dims=actor_dims))
        
        self.obs_encoder = nn.Sequential(*mlp_factory(activation=activation,
                                 input_dims=num_prop,
                                 out_dims=latent_dim,
                                 hidden_dims=obs_encoder_dims))
        
        self.bn = nn.BatchNorm1d(latent_dim,affine=False)

    def forward(self,obs,obs_hist):
        obs_hist_full = torch.cat([
                obs_hist[:,1:,:],
                obs.unsqueeze(1)
            ], dim=1)
        latents = self.rnn_encoder(obs_hist_full)
        actor_input = torch.cat([latents,obs],dim=-1)
        mean  = self.actor(actor_input)
        return mean
    
    def BarlowTwinsLoss(self,obs,obs_hist,weight):
        b = obs.size()[0]
        hist_latent = self.rnn_encoder(obs_hist)
        obs_latent = self.obs_encoder(obs)

        c = self.bn(hist_latent).T @ self.bn(obs_latent)
        c.div_(b)
        # c = torch.sum(c,dim=0,keepdim=True)

        on_diag = torch.diagonal(c).add_(-1).pow_(2).sum()
        off_diag = off_diagonal(c).pow_(2).sum()
        loss = on_diag + weight*off_diag
        return loss
    
class MlpBarlowTwinsActor(nn.Module):
    def __init__(self,
                 num_prop,
                 num_hist,
                 obs_encoder_dims,
                 mlp_encoder_dims,
                 actor_dims,
                 latent_dim,
                 num_actions,
                 activation) -> None:
        super(MlpBarlowTwinsActor,self).__init__()
        self.mlp_encoder = nn.Sequential(*mlp_layernorm_factory(activation=activation,
                                 input_dims=num_prop*num_hist,
                                 out_dims=latent_dim+10,
                                 hidden_dims=mlp_encoder_dims))

        self.actor = nn.Sequential(*mlp_factory(activation=activation,
                                 input_dims=latent_dim + num_prop + 10,
                                 out_dims=num_actions,
                                 hidden_dims=actor_dims))
        
        self.obs_encoder = nn.Sequential(*mlp_layernorm_factory(activation=activation,
                                 input_dims=num_prop,
                                 out_dims=latent_dim,
                                 hidden_dims=obs_encoder_dims))
        
        self.bn = nn.BatchNorm1d(latent_dim,affine=False)

    def forward(self,obs,obs_hist):
        # with torch.no_grad():
        obs_hist_full = torch.cat([
                obs_hist[:,1:,:],
                obs.unsqueeze(1)
            ], dim=1)
        b,_,_ = obs_hist_full.size()
        obs_hist_full = obs_hist_full[:,0:,:].view(b,-1)
        latents = self.mlp_encoder(obs_hist_full)
        actor_input = torch.cat([latents,obs],dim=-1)
        mean  = self.actor(actor_input)
        return mean

    
    def BarlowTwinsLoss(self,obs,obs_hist,priv,weight):
        b = obs.size()[0]
        obs_hist = obs_hist[:,0:,:].view(b,-1)
        predicted = self.mlp_encoder(obs_hist)
        hist_latent = predicted[:,10:]
        priv_latent = predicted[:,:10]

        obs_latent = self.obs_encoder(obs)

        c = self.bn(hist_latent).T @ self.bn(obs_latent)
        c.div_(b)

        on_diag = torch.diagonal(c).add_(-1).pow_(2).sum()
        off_diag = off_diagonal(c).pow_(2).sum()

        priv_loss = F.mse_loss(priv_latent,priv)
        loss = on_diag + weight*off_diag + 0.01*priv_loss
        return loss

class TransBarlowTwinsActor(nn.Module):
    def __init__(self,
                 num_prop,
                 obs_encoder_dims,
                 actor_dims,
                 latent_dim,
                 num_actions,
                 activation) -> None:
        super(TransBarlowTwinsActor,self).__init__()
        self.transformer_config = Config()
        self.transformer_config.n_action=latent_dim + 7
        self.transformer_config.n_layer = 2
        
        self.trans_encoder = StateCausalTransformer(self.transformer_config)

        self.actor = nn.Sequential(*mlp_factory(activation=activation,
                                 input_dims=latent_dim + num_prop + 7,
                                 out_dims=num_actions,
                                 hidden_dims=actor_dims))
        
        self.obs_encoder = nn.Sequential(*mlp_layernorm_factory(activation=activation,
                                 input_dims=num_prop,
                                 out_dims=latent_dim,
                                 hidden_dims=obs_encoder_dims))
        
        self.bn = nn.BatchNorm1d(latent_dim,affine=False)

    def forward(self,obs,obs_hist):
        # with torch.no_grad():
        obs_hist_full = torch.cat([
                obs_hist[:,1:],
                obs.unsqueeze(1)
        ], dim=1)
        latent = self.trans_encoder(obs_hist_full[:,5:,:])
        actor_input = torch.cat([latent,obs],dim=-1)
        mean  = self.actor(actor_input)
        return mean
    
    def BarlowTwinsLoss(self,obs,obs_hist,priv,weight):
        b = obs.size()[0]
        predicted = self.trans_encoder(obs_hist[:,5:,:])
        hist_latent = predicted[:,7:]
        priv_latent = predicted[:,:7]

        obs_latent = self.obs_encoder(obs)

        c = self.bn(hist_latent).T @ self.bn(obs_latent)
        c.div_(b)

        on_diag = torch.diagonal(c).add_(-1).pow_(2).sum()
        off_diag = off_diagonal(c).pow_(2).sum()

        priv_loss = F.mse_loss(priv_latent,priv)
        loss = on_diag + weight*off_diag + 0.01*priv_loss
        return loss

def off_diagonal(x):
    n,m = x.shape
    assert n==m
    return x.flatten()[:-1].view(n-1,n+1)[:,1:].flatten()

class AeActor(nn.Module):
    def __init__(self,
                 num_prop,
                 num_hist,
                 encoder_dims,
                 decoder_dims,
                 actor_dims,
                 num_actions,
                 activation,
                 latent_dim) -> None:
        super(AeActor,self).__init__()
        self.ae = AutoEncoder(activation_fn=activation,
                            input_size=num_prop*num_hist,
                            encoder_dims=encoder_dims,
                            decoder_dims=decoder_dims,
                            latent_dim=latent_dim,
                            output_size=num_prop)
        
        self.actor = nn.Sequential(*mlp_factory(activation=activation,
                                 input_dims=latent_dim + num_prop,
                                 out_dims=num_actions,
                                 hidden_dims=actor_dims))

    def forward(self,obs,obs_hist):
        # self.rnn_encoder.reset_hidden()
        obs_hist_full = torch.cat([
                obs_hist[:,1:],
                obs.unsqueeze(1)
            ], dim=1)
        b,t,n = obs_hist_full.size()
        obs_hist_full = obs_hist_full.view(b,-1)
        latent = self.ae.encode(obs_hist_full)
        actor_input = torch.cat([latent,obs],dim=-1)
        mean  = self.actor(actor_input)
        return mean

    def predict_next_state(self,obs_hist):
        b,t,n = obs_hist.size()
        obs_hist_flatten = obs_hist.view(b,-1)
        latent = self.ae.encode(obs_hist_flatten)
        predicted = self.ae.decode(latent)
        return predicted,latent
        
class StateCausalTransformerActor(nn.Module):
    def __init__(self):
        super(StateCausalTransformerActor,self).__init__()
        self.transformer_config = Config()
        self.transformer = StateCausalTransformer(self.transformer_config)

    def forward(self,obs,obs_hist):
        obs_hist_full = torch.cat([
                obs_hist[:,1:],
                obs.unsqueeze(1)
            ], dim=1)
        
        predicted_state = self.transformer(obs_hist_full[:,5:,:])
        action = predicted_state[:,36:]
        return action
    
    def predict_next_state(self,obs_hist):
        predicted_state = self.transformer(obs_hist[:,5:,:])
        return predicted_state

class ActorCriticRMA(nn.Module):
    is_recurrent = False
    def __init__(self,  num_prop,
                        num_scan,
                        num_critic_obs,
                        num_priv_latent, 
                        num_hist,
                        num_actions,
                        scan_encoder_dims=[256, 256, 256],
                        actor_hidden_dims=[256, 256, 256],
                        critic_hidden_dims=[256, 256, 256],
                        activation='elu',
                        init_noise_std=1.0,
                        **kwargs):
        super(ActorCriticRMA, self).__init__()

        self.kwargs = kwargs
        priv_encoder_dims= kwargs['priv_encoder_dims']
        cost_dims = kwargs['num_costs']
        activation = get_activation(activation)
        self.num_prop = num_prop
        self.num_scan = num_scan
        self.num_hist = num_hist
        self.num_actions = num_actions
        self.num_priv_latent = num_priv_latent
        self.if_scan_encode = scan_encoder_dims is not None and num_scan > 0

        self.teacher_act = kwargs['teacher_act']
        if self.teacher_act:
            print("ppo with teacher actor")
        else:
            print("ppo with student actor")

        self.imi_flag = kwargs['imi_flag']
        if self.imi_flag:
            print("run imitation")
        else:
            print("no imitation")

        if len(priv_encoder_dims) > 0:
            priv_encoder_layers = mlp_factory(activation,num_priv_latent,None,priv_encoder_dims,last_act=True)
            self.priv_encoder = nn.Sequential(*priv_encoder_layers)
            priv_encoder_output_dim = priv_encoder_dims[-1]
        else:
            self.priv_encoder = nn.Identity()
            priv_encoder_output_dim = num_priv_latent

        if self.if_scan_encode:
            scan_encoder_layers = mlp_factory(activation,num_scan,None,scan_encoder_dims,last_act=True)
            self.scan_encoder = nn.Sequential(*scan_encoder_layers)
            self.scan_encoder_output_dim = scan_encoder_dims[-1]
        else:
            self.scan_encoder = nn.Identity()
            self.scan_encoder_output_dim = num_scan

        self.history_encoder = StateHistoryEncoder(activation, num_prop, num_hist, 32)
        # actor_teacher_layers = mlp_factory(activation,num_prop+priv_encoder_output_dim+self.scan_encoder_output_dim,num_actions,actor_hidden_dims,last_act=False)
        actor_teacher_layers = mlp_factory(activation,num_prop+priv_encoder_output_dim+32,num_actions,actor_hidden_dims,last_act=False)

        self.actor_teacher_backbone = nn.Sequential(*actor_teacher_layers)
        self.actor_student_backbone = CnnActor(num_prop=num_prop,
                                               num_hist=num_hist,
                                               num_actions=num_actions,
                                               priv_encoder_output_dim=priv_encoder_output_dim,
                                               actor_hidden_dims=actor_hidden_dims,
                                               activation=activation)

        # Value function
        critic_layers = mlp_factory(activation,num_prop+self.scan_encoder_output_dim+priv_encoder_output_dim+32,1,critic_hidden_dims,last_act=False)
        self.critic = nn.Sequential(*critic_layers)

        # cost function
        cost_layers = mlp_factory(activation,num_prop+self.scan_encoder_output_dim+priv_encoder_output_dim+32,cost_dims,critic_hidden_dims,last_act=False)
        cost_layers.append(nn.Softplus())
        self.cost = nn.Sequential(*cost_layers)

        # Action noise
        self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))
        self.distribution = None
        # disable args validation for speedup
        Normal.set_default_validate_args = False

    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        [torch.nn.init.orthogonal_(module.weight, gain=scales[idx]) for idx, module in
         enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))]
        
    def set_teacher_act(self,flag):
        self.teacher_act = flag
        if self.teacher_act:
            print("acting by teacher")
        else:
            print("acting by student")

    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError
    
    def get_std(self):
        return self.std
    
    @property
    def action_mean(self):
        return self.distribution.mean

    @property
    def action_std(self):
        return self.distribution.stddev
    
    @property
    def entropy(self):
        return self.distribution.entropy().sum(dim=-1)

    def update_distribution(self, obs):
        if self.teacher_act:
            mean = self.act_teacher(obs)
        else:
            mean = self.act_student(obs)
        self.distribution = Normal(mean, mean*0. + self.get_std())

    def act(self, obs,**kwargs):
        self.update_distribution(obs)
        return self.distribution.sample()
    
    def get_actions_log_prob(self, actions):
        return self.distribution.log_prob(actions).sum(dim=-1)

    def act_student(self, obs, **kwargs):
        obs_prop = obs[:, :self.num_prop]
        hist = obs[:, -self.num_hist*self.num_prop:].view(-1,self.num_hist,self.num_prop)
        mean = self.actor_student_backbone(obs_prop,hist)
        return mean
    
    def act_teacher(self,obs, **kwargs):
        obs_prop = obs[:, :self.num_prop]

        # scan_latent = self.infer_scandots_latent(obs)
        latent = self.infer_priv_latent(obs)
        hist_latent = self.infer_hist_latent(obs)

        backbone_input = torch.cat([obs_prop,latent,hist_latent], dim=1)
        mean = self.actor_teacher_backbone(backbone_input)
        return mean
        
    def evaluate(self, obs, **kwargs):
        obs_prop = obs[:, :self.num_prop]
        
        scan_latent = self.infer_scandots_latent(obs)
        latent = self.infer_priv_latent(obs)
        hist_latent = self.infer_hist_latent(obs)

        backbone_input = torch.cat([obs_prop,latent,scan_latent,hist_latent], dim=1)
        value = self.critic(backbone_input)
        return value
    
    def evaluate_cost(self,obs, **kwargs):
        obs_prop = obs[:, :self.num_prop]
        
        scan_latent = self.infer_scandots_latent(obs)
        latent = self.infer_priv_latent(obs)
        hist_latent = self.infer_hist_latent(obs)

        backbone_input = torch.cat([obs_prop,latent,scan_latent,hist_latent], dim=1)
        value = self.cost(backbone_input)
        return value
    
    def infer_priv_latent(self, obs):
        priv = obs[:, self.num_prop + self.num_scan: self.num_prop + self.num_scan + self.num_priv_latent]
        return self.priv_encoder(priv)
    
    def infer_scandots_latent(self, obs):
        scan = obs[:, self.num_prop:self.num_prop + self.num_scan]
        return self.scan_encoder(scan)
     
    def infer_hist_latent(self, obs):
        hist = obs[:, -self.num_hist*self.num_prop:]
        return self.history_encoder(hist.view(-1, self.num_hist, self.num_prop))
    
    def imitation_learning_loss(self, obs):
        with torch.no_grad():
            target_mean = self.act_teacher(obs)
        mean = self.act_student(obs)

        loss = F.mse_loss(mean,target_mean.detach())
        return loss
    
    def imitation_mode(self):
        self.actor_teacher_backbone.eval()
        self.scan_encoder.eval()
        self.priv_encoder.eval()
    
    def save_torch_jit_policy(self,path,device):
        print("ActorCriticRMA")
        obs_demo_input = torch.randn(1,self.num_prop).to(device)
        hist_demo_input = torch.randn(1,self.num_hist,self.num_prop).to(device)
        model_jit = torch.jit.trace(self.actor_student_backbone,(obs_demo_input,hist_demo_input))
        model_jit.save(path)

class ActorCritic(nn.Module):
    is_recurrent = False
    def __init__(self,  num_actor_obs,
                        num_critic_obs,
                        num_actions,
                        actor_hidden_dims=[256, 256, 256],
                        critic_hidden_dims=[256, 256, 256],
                        activation='elu',
                        init_noise_std=1.0,
                        **kwargs):
        if kwargs:
            print("ActorCritic.__init__ got unexpected arguments, which will be ignored: " + str([key for key in kwargs.keys()]))
        super(ActorCritic, self).__init__()

        activation = get_activation(activation)

        mlp_input_dim_a = num_actor_obs
        mlp_input_dim_c = num_critic_obs

        # Policy
        actor_layers = []
        actor_layers.append(nn.Linear(mlp_input_dim_a, actor_hidden_dims[0]))
        actor_layers.append(activation)
        for l in range(len(actor_hidden_dims)):
            if l == len(actor_hidden_dims) - 1:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], num_actions))
            else:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], actor_hidden_dims[l + 1]))
                actor_layers.append(activation)
        self.actor = nn.Sequential(*actor_layers)

        # Value function
        critic_layers = []
        critic_layers.append(nn.Linear(mlp_input_dim_c, critic_hidden_dims[0]))
        critic_layers.append(activation)
        for l in range(len(critic_hidden_dims)):
            if l == len(critic_hidden_dims) - 1:
                critic_layers.append(nn.Linear(critic_hidden_dims[l], 1))
            else:
                critic_layers.append(nn.Linear(critic_hidden_dims[l], critic_hidden_dims[l + 1]))
                critic_layers.append(activation)
        self.critic = nn.Sequential(*critic_layers)

        print(f"Actor MLP: {self.actor}")
        print(f"Critic MLP: {self.critic}")

        # Action noise
        self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))
        self.distribution = None
        # disable args validation for speedup
        Normal.set_default_validate_args = False
        
        # seems that we get better performance without init
        # self.init_memory_weights(self.memory_a, 0.001, 0.)
        # self.init_memory_weights(self.memory_c, 0.001, 0.)

    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        [torch.nn.init.orthogonal_(module.weight, gain=scales[idx]) for idx, module in
         enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))]


    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError
    
    @property
    def action_mean(self):
        return self.distribution.mean

    @property
    def action_std(self):
        return self.distribution.stddev
    
    @property
    def entropy(self):
        return self.distribution.entropy().sum(dim=-1)

    def update_distribution(self, observations):
        mean = self.actor(observations)
        self.distribution = Normal(mean, mean*0. + self.std)

    def act(self, observations, **kwargs):
        self.update_distribution(observations)
        return self.distribution.sample()
    
    def get_actions_log_prob(self, actions):
        return self.distribution.log_prob(actions).sum(dim=-1)

    def act_inference(self, observations):
        actions_mean = self.actor(observations)
        return actions_mean

    def evaluate(self, critic_observations, **kwargs):
        value = self.critic(critic_observations)
        return value

class ActorCriticBarlowTwins(nn.Module):
    is_recurrent = False
    def __init__(self,  num_prop,
                        num_scan,
                        num_critic_obs,
                        num_priv_latent, 
                        num_hist,
                        num_actions,
                        scan_encoder_dims=[256, 256, 256],
                        actor_hidden_dims=[256, 256, 256],
                        critic_hidden_dims=[256, 256, 256],
                        activation='elu',
                        init_noise_std=1.0,
                        **kwargs):
        super(ActorCriticBarlowTwins, self).__init__()

        self.kwargs = kwargs
        priv_encoder_dims= kwargs['priv_encoder_dims']
        cost_dims = kwargs['num_costs']
        activation = get_activation(activation)
        self.num_prop = num_prop
        self.num_scan = num_scan
        self.num_hist = num_hist
        self.num_actions = num_actions
        self.num_priv_latent = num_priv_latent
        self.if_scan_encode = scan_encoder_dims is not None and num_scan > 0

        self.teacher_act = kwargs['teacher_act']
        if self.teacher_act:
            print("ppo with teacher actor")
        else:
            print("ppo with teacher actor")

        self.imi_flag = kwargs['imi_flag']
        if self.imi_flag:
            print("run imitation")
        else:
            print("no imitation")

        if len(priv_encoder_dims) > 0:
            priv_encoder_layers = mlp_factory(activation,num_priv_latent,None,priv_encoder_dims,last_act=True)
            self.priv_encoder = nn.Sequential(*priv_encoder_layers)
            priv_encoder_output_dim = priv_encoder_dims[-1]
        else:
            self.priv_encoder = nn.Identity()
            priv_encoder_output_dim = num_priv_latent

        if self.if_scan_encode:
            scan_encoder_layers = mlp_factory(activation,num_scan,None,scan_encoder_dims,last_act=True)
            self.scan_encoder = nn.Sequential(*scan_encoder_layers)
            self.scan_encoder_output_dim = scan_encoder_dims[-1]
        else:
            self.scan_encoder = nn.Identity()
            self.scan_encoder_output_dim = num_scan

        self.history_encoder = StateHistoryEncoder(activation, num_prop, num_hist, 32)

        # self.actor_teacher_backbone = RnnBarlowTwinsActor(num_prop=num_prop,
        #                               num_actions=num_actions,
        #                               actor_dims=[512,256,128],
        #                               encoder_output_dim=32,
        #                               hidden_dim=128,
        #                               activation=activation,
        #                               latent_dim=64,
        #                               obs_encoder_dims=[256,128],
        #                               rnn_encoder_dims=[128])
        # #MlpBarlowTwinsActor
        self.actor_teacher_backbone = MlpBarlowTwinsActor(num_prop=num_prop,
                                      num_hist=10,
                                      num_actions=num_actions,
                                      actor_dims=[512,256,128],
                                      mlp_encoder_dims=[512,256,128],
                                      activation=activation,
                                      latent_dim=16,
                                      obs_encoder_dims=[256,128])
        print(self.actor_teacher_backbone)

        # Value function
        critic_layers = mlp_factory(activation,num_prop+self.scan_encoder_output_dim+priv_encoder_output_dim+32,1,critic_hidden_dims,last_act=False)
        self.critic = nn.Sequential(*critic_layers)

        # cost function
        cost_layers = mlp_factory(activation,num_prop+self.scan_encoder_output_dim+priv_encoder_output_dim+32,cost_dims,critic_hidden_dims,last_act=False)
        cost_layers.append(nn.Softplus())
        self.cost = nn.Sequential(*cost_layers)

        # Action noise
        self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))
        self.distribution = None
        # disable args validation for speedup
        Normal.set_default_validate_args = False

    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        [torch.nn.init.orthogonal_(module.weight, gain=scales[idx]) for idx, module in
         enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))]
        
    def set_teacher_act(self,flag):
        self.teacher_act = flag
        if self.teacher_act:
            print("acting by teacher")
        else:
            print("acting by student")

    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError
    
    def get_std(self):
        return self.std
    
    @property
    def action_mean(self):
        return self.distribution.mean

    @property
    def action_std(self):
        return self.distribution.stddev
    
    @property
    def entropy(self):
        return self.distribution.entropy().sum(dim=-1)

    def update_distribution(self, obs):
        mean = self.act_teacher(obs)
        self.distribution = Normal(mean, mean*0. + self.get_std())

    def act(self, obs,**kwargs):
        self.update_distribution(obs)
        return self.distribution.sample()
    
    def get_actions_log_prob(self, actions):
        return self.distribution.log_prob(actions).sum(dim=-1)
    
    def act_teacher(self,obs, **kwargs):
        obs_prop = obs[:, :self.num_prop]
        obs_hist = obs[:, -self.num_hist*self.num_prop:].view(-1, self.num_hist, self.num_prop)
        mean = self.actor_teacher_backbone(obs_prop,obs_hist)
        return mean
        
    def evaluate(self, obs, **kwargs):
        obs_prop = obs[:, :self.num_prop]
        
        scan_latent = self.infer_scandots_latent(obs)
        latent = self.infer_priv_latent(obs)
        history_latent = self.infer_hist_latent(obs)

        backbone_input = torch.cat([obs_prop,latent,scan_latent,history_latent], dim=1)
        value = self.critic(backbone_input)
        return value
    
    def evaluate_cost(self,obs, **kwargs):
        obs_prop = obs[:, :self.num_prop]
        
        scan_latent = self.infer_scandots_latent(obs)
        latent = self.infer_priv_latent(obs)
        history_latent = self.infer_hist_latent(obs)

        backbone_input = torch.cat([obs_prop,latent,scan_latent,history_latent], dim=1)
        value = self.cost(backbone_input)
        return value
    
    def infer_priv_latent(self, obs):
        priv = obs[:, self.num_prop + self.num_scan: self.num_prop + self.num_scan + self.num_priv_latent]
        return self.priv_encoder(priv)
    
    def infer_scandots_latent(self, obs):
        scan = obs[:, self.num_prop:self.num_prop + self.num_scan]
        return self.scan_encoder(scan)
    
    def infer_hist_latent(self, obs):
        hist = obs[:, -self.num_hist*self.num_prop:]
        return self.history_encoder(hist.view(-1, self.num_hist, self.num_prop))
    
    def imitation_learning_loss(self, obs):
        obs_prop = obs[:, :self.num_prop]
        obs_hist = obs[:, -self.num_hist*self.num_prop:].view(-1, self.num_hist, self.num_prop)
        priv = obs[:, self.num_prop + self.num_scan: self.num_prop + self.num_scan + 10]

        loss = self.actor_teacher_backbone.BarlowTwinsLoss(obs_prop,obs_hist,priv,5e-3)
        return loss
    
    def imitation_mode(self):
        pass
    
    def save_torch_jit_policy(self,path,device):
        obs_demo_input = torch.randn(1,self.num_prop).to(device)
        hist_demo_input = torch.randn(1,self.num_hist,self.num_prop).to(device)
        model_jit = torch.jit.trace(self.actor_teacher_backbone,(obs_demo_input,hist_demo_input))
        model_jit.save(path)
        torch_out = torch.onnx.export(self.actor_teacher_backbone,
                            (obs_demo_input,hist_demo_input),
                            "test.onnx",
                            verbose=True,
                            export_params=True
                            )
        